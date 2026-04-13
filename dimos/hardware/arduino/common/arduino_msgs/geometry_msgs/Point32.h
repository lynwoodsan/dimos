/*
 * geometry_msgs/Point32 — Arduino-compatible LCM C encode/decode.
 * Wire format: 3x float (x, y, z), big-endian = 12 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_POINT32_H
#define DIMOS_ARDUINO_MSG_POINT32_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} dimos_msg__Point32;

static inline int dimos_msg__Point32__encoded_size(void) { return 12; }

static inline int dimos_msg__Point32__encode(void *buf, int offset, int maxlen,
                                               const dimos_msg__Point32 *p)
{
    int pos = 0, thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->z, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Point32__decode(const void *buf, int offset,
                                               int maxlen, dimos_msg__Point32 *p)
{
    int pos = 0, thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->z, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif
