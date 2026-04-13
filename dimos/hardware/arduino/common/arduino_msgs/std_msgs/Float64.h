/*
 * std_msgs/Float64 — Arduino-compatible LCM C encode/decode.
 * Wire format: 1x double (8 bytes, big-endian).
 */
#ifndef DIMOS_ARDUINO_MSG_FLOAT64_H
#define DIMOS_ARDUINO_MSG_FLOAT64_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double data;
} dimos_msg__Float64;

static inline int dimos_msg__Float64__encoded_size(void) { return 8; }

static inline int dimos_msg__Float64__encode(void *buf, int offset, int maxlen,
                                          const dimos_msg__Float64 *p)
{
    return __double_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Float64__decode(const void *buf, int offset, int maxlen,
                                          dimos_msg__Float64 *p)
{
    return __double_decode_array(buf, offset, maxlen, &p->data, 1);
}

#ifdef __cplusplus
}
#endif

#endif
