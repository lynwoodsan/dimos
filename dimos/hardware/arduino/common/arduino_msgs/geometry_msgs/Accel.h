/*
 * geometry_msgs/Accel — Arduino-compatible LCM C encode/decode.
 * Wire format: Vector3(24) + Vector3(24) = 48 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_ACCEL_H
#define DIMOS_ARDUINO_MSG_ACCEL_H

#include "geometry_msgs/Vector3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Vector3 linear;
    dimos_msg__Vector3 angular;
} dimos_msg__Accel;

static inline int dimos_msg__Accel__encoded_size(void) { return 48; }

static inline int dimos_msg__Accel__encode(void *buf, int offset, int maxlen,
                                             const dimos_msg__Accel *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->linear);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->angular);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Accel__decode(const void *buf, int offset,
                                             int maxlen, dimos_msg__Accel *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->linear);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->angular);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif
