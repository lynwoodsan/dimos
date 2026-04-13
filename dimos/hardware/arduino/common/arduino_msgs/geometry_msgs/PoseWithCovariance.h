/*
 * geometry_msgs/PoseWithCovariance — Arduino-compatible LCM C encode/decode.
 * Wire format: Pose(56) + 36x double(288) = 344 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_POSEWITHCOVARIANCE_H
#define DIMOS_ARDUINO_MSG_POSEWITHCOVARIANCE_H

#include "geometry_msgs/Pose.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dimos_msg__Pose pose;
    double covariance[36];
} dimos_msg__PoseWithCovariance;

static inline int dimos_msg__PoseWithCovariance__encoded_size(void) { return 344; }

static inline int dimos_msg__PoseWithCovariance__encode(
    void *buf, int offset, int maxlen, const dimos_msg__PoseWithCovariance *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__encode(buf, offset + pos, maxlen - pos, &p->pose);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_encode_array(buf, offset + pos, maxlen - pos, p->covariance, 36);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__PoseWithCovariance__decode(
    const void *buf, int offset, int maxlen, dimos_msg__PoseWithCovariance *p)
{
    int pos = 0, thislen;
    thislen = dimos_msg__Pose__decode(buf, offset + pos, maxlen - pos, &p->pose);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __double_decode_array(buf, offset + pos, maxlen - pos, p->covariance, 36);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif
