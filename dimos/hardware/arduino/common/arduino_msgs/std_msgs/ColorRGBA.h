/*
 * std_msgs/ColorRGBA — Arduino-compatible LCM C encode/decode.
 * Wire format: 4x float (r, g, b, a), big-endian = 16 bytes.
 */
#ifndef DIMOS_ARDUINO_MSG_COLORRGBA_H
#define DIMOS_ARDUINO_MSG_COLORRGBA_H

#include "lcm_coretypes_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float r;
    float g;
    float b;
    float a;
} dimos_msg__ColorRGBA;

static inline int dimos_msg__ColorRGBA__encoded_size(void) { return 16; }

static inline int dimos_msg__ColorRGBA__encode(void *buf, int offset, int maxlen,
                                            const dimos_msg__ColorRGBA *p)
{
    int pos = 0, thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->r, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->g, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->b, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &p->a, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

static inline int dimos_msg__ColorRGBA__decode(const void *buf, int offset, int maxlen,
                                            dimos_msg__ColorRGBA *p)
{
    int pos = 0, thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->r, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->g, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->b, 1);
    if (thislen < 0) return thislen; pos += thislen;
    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &p->a, 1);
    if (thislen < 0) return thislen; pos += thislen;
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif
