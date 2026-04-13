/*
 * std_msgs/Bool — Arduino-compatible LCM C encode/decode.
 * Wire format: 1x int8_t, big-endian = 1 byte.
 */
#ifndef DIMOS_ARDUINO_MSG_BOOL_H
#define DIMOS_ARDUINO_MSG_BOOL_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int8_t data;
} dimos_msg__Bool;

static inline int dimos_msg__Bool__encoded_size(void) { return 1; }

static inline int dimos_msg__Bool__encode(void *buf, int offset, int maxlen,
                                       const dimos_msg__Bool *p)
{
    return __int8_t_encode_array(buf, offset, maxlen, &p->data, 1);
}

static inline int dimos_msg__Bool__decode(const void *buf, int offset, int maxlen,
                                       dimos_msg__Bool *p)
{
    return __int8_t_decode_array(buf, offset, maxlen, &p->data, 1);
}

#ifdef __cplusplus
}
#endif

#endif
