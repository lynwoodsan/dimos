/*
 * std_msgs/Time — Arduino-compatible LCM C encode/decode.
 * Wire format: 2x int32_t (sec, nsec), big-endian = 8 bytes.
 * Auto-generated layout — do not edit.
 */
#ifndef DIMOS_ARDUINO_MSG_TIME_H
#define DIMOS_ARDUINO_MSG_TIME_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t sec;
    int32_t nsec;
} dimos_msg__Time;

static inline int dimos_msg__Time__encoded_size(void) { return 8; }

static inline int dimos_msg__Time__encode(void *buf, int offset, int maxlen,
                                       const dimos_msg__Time *p)
{
    int pos = 0, thislen;
    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &p->sec, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &p->nsec, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Time__decode(const void *buf, int offset, int maxlen,
                                       dimos_msg__Time *p)
{
    int pos = 0, thislen;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &p->sec, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &p->nsec, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif
