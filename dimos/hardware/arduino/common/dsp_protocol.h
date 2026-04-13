/*
 * dsp_protocol.h — DimOS Serial Protocol (DSP)
 *
 * Framed binary protocol for Arduino ↔ Host communication over USB serial.
 *
 * Frame format:
 *   [0xD1] [TOPIC 1B] [LENGTH 2B LE] [PAYLOAD 0-1024B] [CRC8 1B]
 *
 * Topic 0 is always DEBUG (UTF-8 text from Serial.print shim).
 * Topics 1..N are data streams, assigned by the generated dimos_arduino.h.
 *
 * This file provides:
 *   - CRC-8/MAXIM table + computation
 *   - dimos_init(baud)
 *   - dimos_send(topic, data, len)
 *   - dimos_poll()
 *   - dimos_on_receive(topic, handler)
 *   - DimosSerial class (Serial.print shim → debug frames)
 *
 * Copyright 2025-2026 Dimensional Inc.  Apache-2.0.
 */

#ifndef DIMOS_DSP_PROTOCOL_H
#define DIMOS_DSP_PROTOCOL_H

#include <stdint.h>
#include <string.h>

/* ======================================================================
 * Constants
 * ====================================================================== */

#define DSP_START_BYTE    0xD1
#define DSP_MAX_PAYLOAD   1024
#define DSP_TOPIC_DEBUG   0
#define DSP_HEADER_SIZE   4   /* START + TOPIC + LENGTH(2) */
#define DSP_OVERHEAD      5   /* HEADER + CRC8 */

/* Maximum number of topic handlers */
#ifndef DSP_MAX_TOPICS
#define DSP_MAX_TOPICS    16
#endif

/* Debug buffer size (Serial.print lines) */
#ifndef DSP_DEBUG_BUF_SIZE
#define DSP_DEBUG_BUF_SIZE 128
#endif

/* ======================================================================
 * CRC-8/MAXIM (polynomial 0x31, init 0x00)
 *
 * Lookup table — 256 bytes of flash on AVR.
 * ====================================================================== */

#ifdef __AVR__
#include <avr/pgmspace.h>
#define DSP_CRC_TABLE_ATTR PROGMEM
#define DSP_CRC_READ(addr) pgm_read_byte(addr)
#else
#define DSP_CRC_TABLE_ATTR
#define DSP_CRC_READ(addr) (*(addr))
#endif

static const uint8_t _dsp_crc8_table[256] DSP_CRC_TABLE_ATTR = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

static inline uint8_t dsp_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    uint16_t i;
    for (i = 0; i < len; i++) {
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ data[i]]);
    }
    return crc;
}

/* ======================================================================
 * Platform abstraction (Arduino vs host C++)
 *
 * On Arduino: uses HardwareSerial directly.
 * On host (for testing): stubs can be provided.
 * ====================================================================== */

#ifdef ARDUINO

/* ======================================================================
 * Arduino Implementation — direct USART register access (polled)
 *
 * We bypass Arduino's HardwareSerial entirely.  HardwareSerial uses
 * interrupt-driven TX which doesn't work in QEMU's AVR USART model
 * and adds buffering/latency on real hardware.  Direct register access
 * is faster, smaller, and works in any AVR simulator.
 *
 * Currently supports USART0 on ATmega328P/2560/etc.  Other AVRs would
 * need conditional register names.
 * ====================================================================== */

#include <Arduino.h>
#include <avr/io.h>

/* --- Direct USART0 helpers --- */

static inline void _dsp_usart_init(uint32_t baud) {
    /* UBRR = F_CPU / (16 * baud) - 1, with U2X=0 */
    /* For higher baud accuracy, use U2X=1: UBRR = F_CPU / (8 * baud) - 1 */
    uint16_t ubrr = (uint16_t)((F_CPU / 8 / baud) - 1);
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);
    UCSR0A = (1 << U2X0);                      /* Double speed */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);      /* Enable RX and TX */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);    /* 8 data bits, 1 stop, no parity */
}

static inline void _dsp_usart_write(uint8_t b) {
    while (!(UCSR0A & (1 << UDRE0))) { /* wait for empty TX buffer */ }
    UDR0 = b;
}

static inline bool _dsp_usart_available(void) {
    return (UCSR0A & (1 << RXC0)) != 0;
}

static inline uint8_t _dsp_usart_read(void) {
    return UDR0;
}

/* --- Internal state --- */
static uint8_t _dsp_rx_buf[DSP_MAX_PAYLOAD];
static bool    _dsp_msg_ready = false;

/* Parser states */
enum _dsp_parse_state {
    DSP_WAIT_START,
    DSP_READ_TOPIC,
    DSP_READ_LEN_LO,
    DSP_READ_LEN_HI,
    DSP_READ_PAYLOAD,
    DSP_READ_CRC
};

static enum _dsp_parse_state _dsp_state = DSP_WAIT_START;
static uint8_t  _dsp_rx_topic;
static uint16_t _dsp_rx_len;
static uint16_t _dsp_rx_payload_pos;

/**
 * Initialize DimOS serial protocol.
 * Call this in setup() before any other dimos_* calls.
 */
static inline void dimos_init(uint32_t baud)
{
    _dsp_usart_init(baud);
    _dsp_state = DSP_WAIT_START;
    _dsp_msg_ready = false;
}

/**
 * Send a DSP frame.
 *
 * @param topic  Topic enum value (DIMOS_TOPIC_DEBUG, DIMOS_TOPIC__*, etc.)
 * @param data   Payload bytes (LCM-encoded for data topics, UTF-8 for debug)
 * @param len    Payload length in bytes
 */
static inline void dimos_send(enum dimos_topic topic, const uint8_t *data, uint16_t len)
{
    if (len > DSP_MAX_PAYLOAD) return;

    /* Build header: START, TOPIC, LENGTH (LE) */
    uint8_t header[DSP_HEADER_SIZE];
    header[0] = DSP_START_BYTE;
    header[1] = (uint8_t)topic;
    header[2] = (uint8_t)(len & 0xFF);
    header[3] = (uint8_t)((len >> 8) & 0xFF);

    /* CRC over TOPIC + LENGTH + PAYLOAD */
    uint8_t crc = 0x00;
    uint16_t i;
    for (i = 1; i < DSP_HEADER_SIZE; i++) {
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ header[i]]);
    }
    for (i = 0; i < len; i++) {
        crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ data[i]]);
    }

    /* Write frame byte-by-byte via direct USART */
    uint16_t k;
    for (k = 0; k < DSP_HEADER_SIZE; k++) _dsp_usart_write(header[k]);
    for (k = 0; k < len; k++)              _dsp_usart_write(data[k]);
    _dsp_usart_write(crc);
}

/**
 * Check for the next incoming DSP message.
 *
 * Reads available serial bytes and attempts to parse a complete frame.
 * Returns true if a valid message is ready.  Use dimos_message_topic(),
 * dimos_message_data(), and dimos_message_len() to access it.
 *
 * Typical usage in loop():
 *
 *   while (dimos_check_message()) {
 *       switch (dimos_message_topic()) {
 *       case DIMOS_TOPIC__MY_INPUT:
 *           MyType msg;
 *           MyType_decode(dimos_message_data(), 0, dimos_message_len(), &msg);
 *           // use msg...
 *           break;
 *       }
 *   }
 */
static inline bool dimos_check_message(void)
{
    /* If a previous message is still unconsumed, clear it */
    _dsp_msg_ready = false;

    while (_dsp_usart_available()) {
        uint8_t b = _dsp_usart_read();

        switch (_dsp_state) {
        case DSP_WAIT_START:
            if (b == DSP_START_BYTE) {
                _dsp_state = DSP_READ_TOPIC;
            }
            break;

        case DSP_READ_TOPIC:
            _dsp_rx_topic = b;
            _dsp_state = DSP_READ_LEN_LO;
            break;

        case DSP_READ_LEN_LO:
            _dsp_rx_len = b;
            _dsp_state = DSP_READ_LEN_HI;
            break;

        case DSP_READ_LEN_HI:
            _dsp_rx_len |= ((uint16_t)b << 8);
            if (_dsp_rx_len > DSP_MAX_PAYLOAD) {
                _dsp_state = DSP_WAIT_START;
                break;
            }
            _dsp_rx_payload_pos = 0;
            if (_dsp_rx_len == 0) {
                _dsp_state = DSP_READ_CRC;
            } else {
                _dsp_state = DSP_READ_PAYLOAD;
            }
            break;

        case DSP_READ_PAYLOAD:
            _dsp_rx_buf[_dsp_rx_payload_pos++] = b;
            if (_dsp_rx_payload_pos >= _dsp_rx_len) {
                _dsp_state = DSP_READ_CRC;
            }
            break;

        case DSP_READ_CRC: {
            /* Verify CRC over topic + length + payload */
            uint8_t crc_input[3];
            crc_input[0] = _dsp_rx_topic;
            crc_input[1] = (uint8_t)(_dsp_rx_len & 0xFF);
            crc_input[2] = (uint8_t)((_dsp_rx_len >> 8) & 0xFF);

            uint8_t crc = dsp_crc8(crc_input, 3);
            if (_dsp_rx_len > 0) {
                /* Continue CRC over payload */
                uint16_t k;
                for (k = 0; k < _dsp_rx_len; k++) {
                    crc = DSP_CRC_READ(&_dsp_crc8_table[crc ^ _dsp_rx_buf[k]]);
                }
            }

            _dsp_state = DSP_WAIT_START;

            if (crc == b) {
                _dsp_msg_ready = true;
                return true;  /* message ready — caller reads it */
            }
            /* CRC mismatch — discard, keep parsing */
            break;
        }
        }
    }

    return false;  /* no complete message available */
}

/**
 * Get the topic of the last received message.
 * Only valid after dimos_check_message() returned true.
 */
static inline enum dimos_topic dimos_message_topic(void)
{
    return (enum dimos_topic)_dsp_rx_topic;
}

/**
 * Get a pointer to the payload of the last received message.
 * Only valid after dimos_check_message() returned true.
 */
static inline const uint8_t *dimos_message_data(void)
{
    return _dsp_rx_buf;
}

/**
 * Get the payload length of the last received message.
 * Only valid after dimos_check_message() returned true.
 */
static inline uint16_t dimos_message_len(void)
{
    return _dsp_rx_len;
}

/* ======================================================================
 * Serial.print shim
 *
 * Intercepts Serial.print/println and sends output as DSP debug frames
 * (topic 0).  Flushes on newline or buffer full.
 * ====================================================================== */

class DimosSerial_ : public Print {
public:
    size_t write(uint8_t b) override {
        _buf[_pos++] = b;
        if (b == '\n' || _pos >= DSP_DEBUG_BUF_SIZE) {
            _flush();
        }
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        size_t i;
        for (i = 0; i < size; i++) {
            write(buffer[i]);
        }
        return size;
    }

    void flush() {
        if (_pos > 0) _flush();
    }

private:
    uint8_t _buf[DSP_DEBUG_BUF_SIZE];
    uint8_t _pos = 0;

    void _flush() {
        dimos_send(DSP_TOPIC_DEBUG, _buf, _pos);
        _pos = 0;
    }
};

static DimosSerial_ DimosSerial;

/*
 * Redirect Serial → DimosSerial so user's Serial.print/println
 * goes through the DSP debug channel instead of raw serial.
 *
 * To access the raw HardwareSerial, use Serial0 (on most boards)
 * or cast: ((HardwareSerial&)Serial0).
 */
#undef Serial
#define Serial DimosSerial

#endif /* ARDUINO */

/* ======================================================================
 * Host-side (C++) utilities
 *
 * The C++ bridge doesn't use dimos_init/dimos_send/dimos_poll (it has
 * its own implementation with termios).  But it shares the constants
 * and CRC function.
 * ====================================================================== */

#endif /* DIMOS_DSP_PROTOCOL_H */
