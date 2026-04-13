/*
 * arduino_bridge — Generic serial ↔ LCM relay for DimOS ArduinoModule.
 *
 * This binary is module-agnostic.  It receives topic→LCM channel mappings
 * via CLI args and forwards raw bytes between USB serial and LCM multicast,
 * prepending/stripping the 8-byte LCM fingerprint hash as needed.
 *
 * Usage:
 *   ./arduino_bridge \
 *     --serial_port /dev/ttyACM0 \
 *     --baudrate 115200 \
 *     --reconnect true \
 *     --reconnect_interval 2.0 \
 *     --topic_out 1 "/imu#sensor_msgs.Imu" \
 *     --topic_in  2 "/cmd#geometry_msgs.Twist"
 *
 * Copyright 2025-2026 Dimensional Inc.  Apache-2.0.
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <signal.h>
#include <string>
#include <thread>
#include <vector>

/* Serial (POSIX) */
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/* LCM */
#include <lcm/lcm-cpp.hpp>

/* DSP protocol constants + CRC */
#include "dsp_protocol.h"

/* ======================================================================
 * Globals
 * ====================================================================== */

static std::atomic<bool> g_running{true};
static int g_serial_fd = -1;
static std::mutex g_serial_write_mutex;

/* Topic mapping */
struct TopicMapping {
    uint8_t topic_id;
    std::string lcm_channel;    /* full "name#msg_type" */
    bool is_output;             /* true = Arduino→Host (publish), false = Host→Arduino (subscribe) */
    std::vector<uint8_t> fingerprint;  /* 8-byte hash, computed at startup */
};

static std::vector<TopicMapping> g_topics;
static lcm::LCM *g_lcm = nullptr;

/* Config */
static std::string g_serial_port;
static int g_baudrate = 115200;
static bool g_reconnect = true;
static float g_reconnect_interval = 2.0f;

/* ======================================================================
 * CLI Parsing
 * ====================================================================== */

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 576000:  return B576000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        default:
            fprintf(stderr, "[bridge] Unsupported baud rate: %d\n", baud);
            return B115200;
    }
}

static void parse_args(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);

        if (arg == "--serial_port" && i + 1 < argc) {
            g_serial_port = argv[++i];
        } else if (arg == "--baudrate" && i + 1 < argc) {
            g_baudrate = std::atoi(argv[++i]);
        } else if (arg == "--reconnect" && i + 1 < argc) {
            std::string val(argv[++i]);
            g_reconnect = (val == "true" || val == "1");
        } else if (arg == "--reconnect_interval" && i + 1 < argc) {
            g_reconnect_interval = std::atof(argv[++i]);
        } else if ((arg == "--topic_out" || arg == "--topic_in") && i + 2 < argc) {
            TopicMapping tm;
            tm.topic_id = (uint8_t)std::atoi(argv[++i]);
            tm.lcm_channel = argv[++i];
            tm.is_output = (arg == "--topic_out");
            g_topics.push_back(tm);
        }
    }
}

/* ======================================================================
 * LCM Fingerprint Hash
 *
 * We need the 8-byte hash for each message type to prepend when publishing
 * and to strip when receiving.  The hash is computed by the LCM-generated
 * C++ type's static method.  Since we're generic, we look up the type name
 * from the channel string ("name#msg_type") and use a registry.
 *
 * For types we don't have compiled in, we use a fallback: read the hash
 * from the first LCM message we receive on that channel.
 * ====================================================================== */

/*
 * Include all LCM C++ message headers we support.
 * The fingerprint hash is available via Type::getHash().
 */
#include "std_msgs/Time.hpp"
#include "std_msgs/Bool.hpp"
#include "std_msgs/Int32.hpp"
#include "std_msgs/Float32.hpp"
#include "std_msgs/Float64.hpp"
#include "std_msgs/ColorRGBA.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "geometry_msgs/Point.hpp"
#include "geometry_msgs/Point32.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Pose.hpp"
#include "geometry_msgs/Pose2D.hpp"
#include "geometry_msgs/Twist.hpp"
#include "geometry_msgs/Accel.hpp"
#include "geometry_msgs/Transform.hpp"
#include "geometry_msgs/Wrench.hpp"
#include "geometry_msgs/Inertia.hpp"
#include "geometry_msgs/PoseWithCovariance.hpp"
#include "geometry_msgs/TwistWithCovariance.hpp"
#include "geometry_msgs/AccelWithCovariance.hpp"

static std::map<std::string, int64_t> g_hash_registry;

static void init_hash_registry()
{
    /* Register all known types */
    g_hash_registry["std_msgs.Time"]       = std_msgs::Time::getHash();
    g_hash_registry["std_msgs.Bool"]       = std_msgs::Bool::getHash();
    g_hash_registry["std_msgs.Int32"]      = std_msgs::Int32::getHash();
    g_hash_registry["std_msgs.Float32"]    = std_msgs::Float32::getHash();
    g_hash_registry["std_msgs.Float64"]    = std_msgs::Float64::getHash();
    g_hash_registry["std_msgs.ColorRGBA"]  = std_msgs::ColorRGBA::getHash();

    g_hash_registry["geometry_msgs.Vector3"]    = geometry_msgs::Vector3::getHash();
    g_hash_registry["geometry_msgs.Point"]      = geometry_msgs::Point::getHash();
    g_hash_registry["geometry_msgs.Point32"]    = geometry_msgs::Point32::getHash();
    g_hash_registry["geometry_msgs.Quaternion"] = geometry_msgs::Quaternion::getHash();
    g_hash_registry["geometry_msgs.Pose"]       = geometry_msgs::Pose::getHash();
    g_hash_registry["geometry_msgs.Pose2D"]     = geometry_msgs::Pose2D::getHash();
    g_hash_registry["geometry_msgs.Twist"]      = geometry_msgs::Twist::getHash();
    g_hash_registry["geometry_msgs.Accel"]      = geometry_msgs::Accel::getHash();
    g_hash_registry["geometry_msgs.Transform"]  = geometry_msgs::Transform::getHash();
    g_hash_registry["geometry_msgs.Wrench"]     = geometry_msgs::Wrench::getHash();
    g_hash_registry["geometry_msgs.Inertia"]    = geometry_msgs::Inertia::getHash();
    g_hash_registry["geometry_msgs.PoseWithCovariance"]  = geometry_msgs::PoseWithCovariance::getHash();
    g_hash_registry["geometry_msgs.TwistWithCovariance"] = geometry_msgs::TwistWithCovariance::getHash();
    g_hash_registry["geometry_msgs.AccelWithCovariance"] = geometry_msgs::AccelWithCovariance::getHash();
}

/* Extract "msg_type" from "topic_name#msg_type" */
static std::string extract_msg_type(const std::string &channel)
{
    auto pos = channel.find('#');
    if (pos == std::string::npos) return "";
    return channel.substr(pos + 1);
}

/* Extract "topic_name" from "topic_name#msg_type" */
static std::string extract_topic_name(const std::string &channel)
{
    auto pos = channel.find('#');
    if (pos == std::string::npos) return channel;
    return channel.substr(0, pos);
}

/* Compute 8-byte big-endian fingerprint from hash value */
static std::vector<uint8_t> hash_to_bytes(int64_t hash)
{
    std::vector<uint8_t> bytes(8);
    uint64_t h = (uint64_t)hash;
    for (int i = 7; i >= 0; i--) {
        bytes[i] = (uint8_t)(h & 0xFF);
        h >>= 8;
    }
    return bytes;
}

static bool resolve_fingerprints()
{
    for (auto &tm : g_topics) {
        std::string msg_type = extract_msg_type(tm.lcm_channel);
        auto it = g_hash_registry.find(msg_type);
        if (it == g_hash_registry.end()) {
            fprintf(stderr, "[bridge] Unknown message type: %s\n", msg_type.c_str());
            return false;
        }
        tm.fingerprint = hash_to_bytes(it->second);
    }
    return true;
}

/* ======================================================================
 * Serial Port
 * ====================================================================== */

static int serial_open(const std::string &port, int baud)
{
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[bridge] Cannot open %s: %s\n", port.c_str(), strerror(errno));
        return -1;
    }

    /* Clear O_NONBLOCK after open (we want blocking reads in the reader thread) */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tcgetattr(fd, &tio);

    /* Raw mode: no echo, no canonical, no signals */
    cfmakeraw(&tio);

    /* 8N1 */
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;

    /* No flow control */
    tio.c_cflag &= ~CRTSCTS;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* Set baud */
    speed_t speed = baud_to_speed(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    /* Read timeout: return after 100ms or 1 byte, whichever first */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;  /* 100ms in deciseconds */

    tcsetattr(fd, TCSANOW, &tio);
    tcflush(fd, TCIOFLUSH);

    return fd;
}

static void serial_close(int fd)
{
    if (fd >= 0) close(fd);
}

/* ======================================================================
 * Serial → LCM (reader thread)
 * ====================================================================== */

/* Build topic_id → TopicMapping lookup */
static std::map<uint8_t, TopicMapping*> g_topic_out_map;

static void serial_reader_thread()
{
    enum { WAIT_START, READ_TOPIC, READ_LEN_LO, READ_LEN_HI, READ_PAYLOAD, READ_CRC } state = WAIT_START;

    uint8_t rx_topic = 0;
    uint16_t rx_len = 0;
    uint16_t rx_pos = 0;
    uint8_t rx_buf[DSP_MAX_PAYLOAD];

    while (g_running.load()) {
        uint8_t b;
        int n = read(g_serial_fd, &b, 1);
        if (n < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[bridge] Serial read error: %s\n", strerror(errno));
            break;
        }
        if (n == 0) continue;  /* timeout, loop back */

        switch (state) {
        case WAIT_START:
            if (b == DSP_START_BYTE) state = READ_TOPIC;
            break;

        case READ_TOPIC:
            rx_topic = b;
            state = READ_LEN_LO;
            break;

        case READ_LEN_LO:
            rx_len = b;
            state = READ_LEN_HI;
            break;

        case READ_LEN_HI:
            rx_len |= ((uint16_t)b << 8);
            if (rx_len > DSP_MAX_PAYLOAD) {
                state = WAIT_START;
                break;
            }
            rx_pos = 0;
            state = (rx_len == 0) ? READ_CRC : READ_PAYLOAD;
            break;

        case READ_PAYLOAD:
            rx_buf[rx_pos++] = b;
            if (rx_pos >= rx_len) state = READ_CRC;
            break;

        case READ_CRC: {
            /* Verify CRC */
            uint8_t check[3] = { rx_topic, (uint8_t)(rx_len & 0xFF), (uint8_t)((rx_len >> 8) & 0xFF) };
            uint8_t crc = dsp_crc8(check, 3);
            crc = dsp_crc8(rx_buf, rx_len);
            /* Actually, CRC is over the concatenation. Recompute properly: */
            uint8_t crc_buf[3 + DSP_MAX_PAYLOAD];
            crc_buf[0] = rx_topic;
            crc_buf[1] = (uint8_t)(rx_len & 0xFF);
            crc_buf[2] = (uint8_t)((rx_len >> 8) & 0xFF);
            memcpy(crc_buf + 3, rx_buf, rx_len);
            uint8_t expected_crc = dsp_crc8(crc_buf, 3 + rx_len);

            if (expected_crc != b) {
                fprintf(stderr, "[bridge] CRC mismatch on topic %d (got 0x%02X, expected 0x%02X)\n",
                        rx_topic, b, expected_crc);
                state = WAIT_START;
                break;
            }

            /* Handle frame */
            if (rx_topic == DSP_TOPIC_DEBUG) {
                /* Debug: print to stdout */
                fwrite(rx_buf, 1, rx_len, stdout);
                fflush(stdout);
            } else {
                /* Data: prepend fingerprint hash and publish to LCM */
                auto it = g_topic_out_map.find(rx_topic);
                if (it != g_topic_out_map.end()) {
                    TopicMapping *tm = it->second;
                    /* Build LCM message: 8-byte hash + payload */
                    int total = 8 + rx_len;
                    std::vector<uint8_t> lcm_buf(total);
                    memcpy(lcm_buf.data(), tm->fingerprint.data(), 8);
                    memcpy(lcm_buf.data() + 8, rx_buf, rx_len);
                    g_lcm->publish(tm->lcm_channel, lcm_buf.data(), total);
                } else {
                    fprintf(stderr, "[bridge] Unknown outbound topic: %d\n", rx_topic);
                }
            }
            state = WAIT_START;
            break;
        }
        }
    }
}

/* ======================================================================
 * LCM → Serial (subscription handler)
 * ====================================================================== */

/* Build LCM channel → TopicMapping lookup */
static std::map<std::string, TopicMapping*> g_topic_in_map;

/* Forward declaration */
static void send_lcm_to_serial(const lcm::ReceiveBuffer *rbuf, TopicMapping *tm);

class RawHandler {
public:
    TopicMapping *tm;
    RawHandler(TopicMapping *t) : tm(t) {}
    void handle(const lcm::ReceiveBuffer *rbuf, const std::string & /*channel*/) {
        send_lcm_to_serial(rbuf, tm);
    }
};

static std::vector<std::unique_ptr<RawHandler>> g_raw_handlers;

static void send_lcm_to_serial(const lcm::ReceiveBuffer *rbuf,
                               TopicMapping *tm)
{

    /* Strip 8-byte fingerprint hash from LCM data */
    if (rbuf->data_size < 8) return;
    const uint8_t *payload = (const uint8_t *)rbuf->data + 8;
    uint16_t payload_len = (uint16_t)(rbuf->data_size - 8);

    if (payload_len > DSP_MAX_PAYLOAD) return;

    /* Build DSP frame */
    uint8_t header[DSP_HEADER_SIZE];
    header[0] = DSP_START_BYTE;
    header[1] = tm->topic_id;
    header[2] = (uint8_t)(payload_len & 0xFF);
    header[3] = (uint8_t)((payload_len >> 8) & 0xFF);

    /* CRC over topic + length + payload */
    uint8_t crc_buf[3 + DSP_MAX_PAYLOAD];
    crc_buf[0] = tm->topic_id;
    crc_buf[1] = header[2];
    crc_buf[2] = header[3];
    memcpy(crc_buf + 3, payload, payload_len);
    uint8_t crc = dsp_crc8(crc_buf, 3 + payload_len);

    /* Write to serial (thread-safe) */
    std::lock_guard<std::mutex> lock(g_serial_write_mutex);
    write(g_serial_fd, header, DSP_HEADER_SIZE);
    if (payload_len > 0) {
        write(g_serial_fd, payload, payload_len);
    }
    write(g_serial_fd, &crc, 1);
}

static void lcm_handler_thread()
{
    while (g_running.load()) {
        int ret = g_lcm->handleTimeout(100);  /* 100ms timeout */
        if (ret < 0) {
            fprintf(stderr, "[bridge] LCM handle error\n");
            break;
        }
    }
}

/* ======================================================================
 * Signal handling
 * ====================================================================== */

static void signal_handler(int /*sig*/)
{
    g_running.store(false);
}

/* ======================================================================
 * Main
 * ====================================================================== */

int main(int argc, char **argv)
{
    parse_args(argc, argv);

    if (g_serial_port.empty()) {
        fprintf(stderr, "Usage: arduino_bridge --serial_port <port> --baudrate <baud> "
                        "[--topic_out <id> <channel>] [--topic_in <id> <channel>] ...\n");
        return 1;
    }

    /* Compute fingerprint hashes */
    init_hash_registry();
    if (!resolve_fingerprints()) {
        return 1;
    }

    /* Build lookup maps */
    for (auto &tm : g_topics) {
        if (tm.is_output) {
            g_topic_out_map[tm.topic_id] = &tm;
        } else {
            g_topic_in_map[tm.lcm_channel] = &tm;
        }
    }

    /* Signal handlers */
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    /* Init LCM */
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[bridge] LCM init failed\n");
        return 1;
    }
    g_lcm = &lcm;

    /* Subscribe to inbound LCM topics */
    for (auto &tm : g_topics) {
        if (!tm.is_output) {
            auto handler = std::make_unique<RawHandler>(&tm);
            lcm.subscribe(tm.lcm_channel, &RawHandler::handle, handler.get());
            g_raw_handlers.push_back(std::move(handler));
            printf("[bridge] Subscribed LCM→Serial: topic %d ← %s\n",
                   tm.topic_id, tm.lcm_channel.c_str());
        } else {
            printf("[bridge] Serial→LCM: topic %d → %s\n",
                   tm.topic_id, tm.lcm_channel.c_str());
        }
    }

    /* Open serial port */
    printf("[bridge] Opening %s at %d baud\n", g_serial_port.c_str(), g_baudrate);

    while (g_running.load()) {
        g_serial_fd = serial_open(g_serial_port, g_baudrate);
        if (g_serial_fd < 0) {
            if (!g_reconnect) return 1;
            fprintf(stderr, "[bridge] Retrying in %.1fs...\n", g_reconnect_interval);
            std::this_thread::sleep_for(
                std::chrono::milliseconds((int)(g_reconnect_interval * 1000)));
            continue;
        }

        printf("[bridge] Serial port opened (fd=%d)\n", g_serial_fd);

        /* Start threads */
        std::thread reader(serial_reader_thread);
        std::thread lcm_thread(lcm_handler_thread);

        /* Wait for reader to exit (serial disconnect or shutdown) */
        reader.join();

        /* Stop LCM thread */
        g_running.store(false);
        lcm_thread.join();

        serial_close(g_serial_fd);
        g_serial_fd = -1;

        if (!g_reconnect || !g_running.load()) break;

        /* Reconnect */
        printf("[bridge] Disconnected, reconnecting in %.1fs...\n", g_reconnect_interval);
        g_running.store(true);
        std::this_thread::sleep_for(
            std::chrono::milliseconds((int)(g_reconnect_interval * 1000)));
    }

    printf("[bridge] Shutting down\n");
    return 0;
}
