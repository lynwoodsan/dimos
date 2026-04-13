/*
 * std_msgs/Float32 — Arduino-compatible LCM C encode/decode.
 * Wire format: 1x float (4 bytes, big-endian).
 */
#ifndef DIMOS_ARDUINO_MSG_FLOAT32_H
#define DIMOS_ARDUINO_MSG_FLOAT32_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float data;
} dimos_msg__Float32;

static inline int dimos_msg__Float32__encoded_size(void) { return 4; }

static inline int dimos_msg__Float32__encode(void *buf, int offset, int maxlen,
                                          const dimos_msg__Float32 *p)
{
    return __float_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Float32__decode(const void *buf, int offset, int maxlen,
                                          dimos_msg__Float32 *p)
{
    return __float_decode_array(buf, offset, maxlen, &p->data, 1);
}

#ifdef __cplusplus
}
#endif

#endif
