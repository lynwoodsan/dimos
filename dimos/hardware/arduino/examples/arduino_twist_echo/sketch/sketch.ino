/*
 * Twist Echo — Example DimOS Arduino sketch.
 *
 * Receives Twist commands from the host, echoes them back.
 * Demonstrates:
 *   - dimos_init() / dimos_check_message() / dimos_send()
 *   - Switch on dimos_message_topic() to handle different streams
 *   - Using generated encode/decode functions
 *   - Serial.println() going through the debug channel
 *   - Config values available as #defines
 *
 * NOTE: We use _delay_ms() from <util/delay.h> instead of Arduino's delay()
 * because delay() relies on timer 0 interrupts which don't fire in QEMU's
 * AVR model.  _delay_ms is a pure busy loop and works in any simulator.
 */

#include "dimos_arduino.h"
#include <util/delay.h>

/* Shared state — accessible across all topic handlers */
dimos_msg__Twist last_twist;
uint32_t msg_count = 0;

void setup() {
    dimos_init(DIMOS_BAUDRATE);
    Serial.println("TwistEcho ready");
}

void loop() {
    while (dimos_check_message()) {
        enum dimos_topic  topic = dimos_message_topic();
        const uint8_t    *data  = dimos_message_data();
        uint16_t          len   = dimos_message_len();

        switch (topic) {

        case DIMOS_TOPIC__EXAMPLE_INPUT_TOPIC1: {
            int decoded = dimos_msg__Twist__decode(data, 0, len, &last_twist);
            if (decoded < 0) {
                Serial.println("ERR: failed to decode Twist");
                break;
            }

            msg_count++;
            Serial.print("Got twist #");
            Serial.print(msg_count);
            Serial.print(": linear.x=");
            Serial.println(last_twist.linear.x);

            /* Echo it back */
            uint8_t buf[48];
            int encoded = dimos_msg__Twist__encode(buf, 0, sizeof(buf), &last_twist);
            if (encoded > 0) {
                dimos_send(DIMOS_TOPIC__EXAMPLE_OUTPUT_TOPIC2, buf, encoded);
            }
            break;
        }

        default:
            break;
        }
    }

    /* _delay_ms requires a compile-time constant */
    _delay_ms(50);
}
