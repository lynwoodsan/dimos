/*
 * geometry_msgs/TwistWithCovariance — Arduino-compatible LCM C encode/decode.
 * Wire format: Twist(48) + 36x double(288) = 336 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_TWISTWITHCOVARIANCE_H
#define DIMOS_ARDUINO_MSG_TWISTWITHCOVARIANCE_H

#include "geometry_msgs/Twist.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Twist twist;
    double covariance[36];
} dimos_msg__TwistWithCovariance;

static inline int dimos_msg__TwistWithCovariance__encoded_size(void) { return 336; }

static inline int dimos_msg__TwistWithCovariance__encode(
    void *buf, int offset, int maxlen, const dimos_msg__TwistWithCovariance *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Twist__encode(buf, offset + pos, maxlen - pos, &p->twist);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, p->covariance, 36);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__TwistWithCovariance__decode(
    const void *buf, int offset, int maxlen, dimos_msg__TwistWithCovariance *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Twist__decode(buf, offset + pos, maxlen - pos, &p->twist);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, p->covariance, 36);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif
