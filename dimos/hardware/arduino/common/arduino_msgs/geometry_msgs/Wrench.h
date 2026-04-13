/*
 * geometry_msgs/Wrench — Arduino-compatible LCM C encode/decode.
 * Wire format: Vector3(24) + Vector3(24) = 48 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_WRENCH_H
#define DIMOS_ARDUINO_MSG_WRENCH_H

#include "geometry_msgs/Vector3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Vector3 force;
    dimos_msg__Vector3 torque;
} dimos_msg__Wrench;

static inline int dimos_msg__Wrench__encoded_size(void) { return 48; }

static inline int dimos_msg__Wrench__encode(void *buf, int offset, int maxlen,
                                              const dimos_msg__Wrench *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->force);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__encode(buf, offset + pos, maxlen - pos, &p->torque);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__Wrench__decode(const void *buf, int offset,
                                              int maxlen, dimos_msg__Wrench *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->force);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = dimos_msg__Vector3__decode(buf, offset + pos, maxlen - pos, &p->torque);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif
