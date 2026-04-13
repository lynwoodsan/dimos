/*
 * geometry_msgs/Quaternion — Arduino-compatible LCM C encode/decode.
 * Wire format: 4x double (x, y, z, w), big-endian = 32 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_QUATERNION_H
#define DIMOS_ARDUINO_MSG_QUATERNION_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double x;
    double y;
    double z;
    double w;
} dimos_msg__Quaternion;

static inline int dimos_msg__Quaternion__encoded_size(void) { return 32; }

static inline int dimos_msg__Quaternion__encode(void *buf, int offset, int maxlen,
                                                  const dimos_msg__Quaternion *p)
{
    int pos = 0, thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->z, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &p->w, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Quaternion__decode(const void *buf, int offset,
                                                  int maxlen, dimos_msg__Quaternion *p)
{
    int pos = 0, thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->x, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->y, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->z, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &p->w, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif
