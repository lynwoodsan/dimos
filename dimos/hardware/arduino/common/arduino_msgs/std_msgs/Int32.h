/*
 * std_msgs/Int32 — Arduino-compatible LCM C encode/decode.
 * Wire format: 1x int32_t, big-endian = 4 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_INT32_H
#define DIMOS_ARDUINO_MSG_INT32_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t data;
} dimos_msg__Int32;

static inline int dimos_msg__Int32__encoded_size(void) { return 4; }

static inline int dimos_msg__Int32__encode(void *buf, int offset, int maxlen,
                                        const dimos_msg__Int32 *p)
{
    return __int32_t_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Int32__decode(const void *buf, int offset, int maxlen,
                                        dimos_msg__Int32 *p)
{
    return __int32_t_decode_array(buf, offset, maxlen, &p->data, 1);
}

#ifdef __cplusplus
}
#endif

#endif
