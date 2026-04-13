/*
 * lcm_coretypes_arduino.h
 *
 * Arduino-compatible LCM primitive encode/decode functions.
 *
 * Binary format is identical to standard LCM wire format (big-endian) for all
 * fixed-size primitives.  The 8-byte fingerprint hash is NOT handled here —
 * the host-side C++ bridge prepends it on publish and strips it on subscribe.
 *
 * Compared to upstream lcm_coretypes.h:
 *   - No malloc / free  (all buffers are caller-provided)
 *   - No string support  (variable-length, requires malloc)
 *   - No variable-length array helpers
 *   - No introspection structs (lcm_field_t, lcm_type_info_t)
 *   - No clone helpers
 *   - double encode/decode works on platforms where sizeof(double)==4
 *     by promoting to/from 8-byte IEEE 754 on the wire
 *
 * Supported types: boolean, byte, int8_t, int16_t, int32_t, int64_t,
 *                  float (4-byte), double (8-byte on wire, 4-byte on AVR)
 *
 * Copyright 2025-2026 Dimensional Inc.  Apache-2.0.
 */

/*
 * Use the same include guard as upstream lcm_coretypes.h so that on
 * x86_64 builds (tests, bridge) whichever header is included first wins.
 * Both produce byte-identical output for fixed-size types on x86_64.
 * On AVR the upstream header is never available, so this is the only one.
 */
#ifndef _LCM_LIB_INLINE_H
#define _LCM_LIB_INLINE_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus

#if defined(__GNUC__) && defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#endif

extern "C" {
#endif

/*
 * Types required by LCM-generated C++ headers.  On AVR these are unused
 * but harmless.  On x86_64 they let C++ headers that reference them compile.
 */
union float_uint32 {
    float f;
    uint32_t i;
};

union double_uint64 {
    double f;
    uint64_t i;
};

typedef struct ___lcm_hash_ptr __lcm_hash_ptr;
struct ___lcm_hash_ptr {
    const __lcm_hash_ptr *parent;
    int64_t (*v)(void);
};

/* ======================================================================
 * BYTE
 * ====================================================================== */

static inline int __byte_encoded_array_size(const uint8_t *p, int elements)
{
    (void) p;
    return (int)(sizeof(uint8_t)) * elements;
}

static inline int __byte_encode_array(void *_buf, int offset, int maxlen,
                                      const uint8_t *p, int elements)
{
    if (maxlen < elements)
        return -1;
    memcpy((uint8_t *)_buf + offset, p, elements);
    return elements;
}

static inline int __byte_decode_array(const void *_buf, int offset, int maxlen,
                                      uint8_t *p, int elements)
{
    if (maxlen < elements)
        return -1;
    memcpy(p, (const uint8_t *)_buf + offset, elements);
    return elements;
}

/* ======================================================================
 * INT8_T  /  BOOLEAN
 * ====================================================================== */

static inline int __int8_t_encoded_array_size(const int8_t *p, int elements)
{
    (void) p;
    return (int)(sizeof(int8_t)) * elements;
}

static inline int __int8_t_encode_array(void *_buf, int offset, int maxlen,
                                        const int8_t *p, int elements)
{
    if (maxlen < elements)
        return -1;
    memcpy((int8_t *)_buf + offset, p, elements);
    return elements;
}

static inline int __int8_t_decode_array(const void *_buf, int offset,
                                        int maxlen, int8_t *p, int elements)
{
    if (maxlen < elements || elements < 0)
        return -1;
    memcpy(p, (const int8_t *)_buf + offset, elements);
    return elements;
}

/* boolean is wire-identical to int8_t */
#define __boolean_encoded_array_size __int8_t_encoded_array_size
#define __boolean_encode_array      __int8_t_encode_array
#define __boolean_decode_array      __int8_t_decode_array

/* ======================================================================
 * INT16_T
 * ====================================================================== */

static inline int __int16_t_encoded_array_size(const int16_t *p, int elements)
{
    (void) p;
    return (int)(sizeof(int16_t)) * elements;
}

static inline int __int16_t_encode_array(void *_buf, int offset, int maxlen,
                                         const int16_t *p, int elements)
{
    int total_size = (int)(sizeof(int16_t)) * elements;
    uint8_t *buf = (uint8_t *)_buf;
    int pos = offset;
    int i;

    if (maxlen < total_size)
        return -1;

    const uint16_t *up = (const uint16_t *)p;
    for (i = 0; i < elements; i++) {
        uint16_t v = up[i];
        buf[pos++] = (uint8_t)((v >> 8) & 0xff);
        buf[pos++] = (uint8_t)(v & 0xff);
    }
    return total_size;
}

static inline int __int16_t_decode_array(const void *_buf, int offset,
                                         int maxlen, int16_t *p, int elements)
{
    int total_size = (int)(sizeof(int16_t)) * elements;
    const uint8_t *buf = (const uint8_t *)_buf;
    int pos = offset;
    int i;

    if (maxlen < total_size)
        return -1;

    for (i = 0; i < elements; i++) {
        p[i] = (int16_t)((buf[pos] << 8) + buf[pos + 1]);
        pos += 2;
    }
    return total_size;
}

/* ======================================================================
 * INT32_T
 * ====================================================================== */

static inline int __int32_t_encoded_array_size(const int32_t *p, int elements)
{
    (void) p;
    return (int)(sizeof(int32_t)) * elements;
}

static inline int __int32_t_encode_array(void *_buf, int offset, int maxlen,
                                         const int32_t *p, int elements)
{
    int total_size = (int)(sizeof(int32_t)) * elements;
    uint8_t *buf = (uint8_t *)_buf;
    int pos = offset;
    int i;

    if (maxlen < total_size)
        return -1;

    const uint32_t *up = (const uint32_t *)p;
    for (i = 0; i < elements; i++) {
        uint32_t v = up[i];
        buf[pos++] = (uint8_t)((v >> 24) & 0xff);
        buf[pos++] = (uint8_t)((v >> 16) & 0xff);
        buf[pos++] = (uint8_t)((v >> 8) & 0xff);
        buf[pos++] = (uint8_t)(v & 0xff);
    }
    return total_size;
}

static inline int __int32_t_decode_array(const void *_buf, int offset,
                                         int maxlen, int32_t *p, int elements)
{
    int total_size = (int)(sizeof(int32_t)) * elements;
    const uint8_t *buf = (const uint8_t *)_buf;
    int pos = offset;
    int i;

    if (maxlen < total_size)
        return -1;

    for (i = 0; i < elements; i++) {
        p[i] = (int32_t)(((uint32_t)buf[pos] << 24) +
                          ((uint32_t)buf[pos + 1] << 16) +
                          ((uint32_t)buf[pos + 2] << 8) +
                          (uint32_t)buf[pos + 3]);
        pos += 4;
    }
    return total_size;
}

/* ======================================================================
 * INT64_T
 * ====================================================================== */

static inline int __int64_t_encoded_array_size(const int64_t *p, int elements)
{
    (void) p;
    return (int)(sizeof(int64_t)) * elements;
}

static inline int __int64_t_encode_array(void *_buf, int offset, int maxlen,
                                         const int64_t *p, int elements)
{
    int total_size = 8 * elements;
    uint8_t *buf = (uint8_t *)_buf;
    int pos = offset;
    int i;

    if (maxlen < total_size)
        return -1;

    const uint64_t *up = (const uint64_t *)p;
    for (i = 0; i < elements; i++) {
        uint64_t v = up[i];
        buf[pos++] = (uint8_t)((v >> 56) & 0xff);
        buf[pos++] = (uint8_t)((v >> 48) & 0xff);
        buf[pos++] = (uint8_t)((v >> 40) & 0xff);
        buf[pos++] = (uint8_t)((v >> 32) & 0xff);
        buf[pos++] = (uint8_t)((v >> 24) & 0xff);
        buf[pos++] = (uint8_t)((v >> 16) & 0xff);
        buf[pos++] = (uint8_t)((v >> 8) & 0xff);
        buf[pos++] = (uint8_t)(v & 0xff);
    }
    return total_size;
}

static inline int __int64_t_decode_array(const void *_buf, int offset,
                                         int maxlen, int64_t *p, int elements)
{
    int total_size = 8 * elements;
    const uint8_t *buf = (const uint8_t *)_buf;
    int pos = offset;
    int i;

    if (maxlen < total_size)
        return -1;

    for (i = 0; i < elements; i++) {
        uint64_t a = (((uint32_t)buf[pos] << 24) +
                      ((uint32_t)buf[pos + 1] << 16) +
                      ((uint32_t)buf[pos + 2] << 8) +
                      (uint32_t)buf[pos + 3]);
        pos += 4;
        uint64_t b = (((uint32_t)buf[pos] << 24) +
                      ((uint32_t)buf[pos + 1] << 16) +
                      ((uint32_t)buf[pos + 2] << 8) +
                      (uint32_t)buf[pos + 3]);
        pos += 4;
        p[i] = (int64_t)((a << 32) + (b & 0xffffffff));
    }
    return total_size;
}

/* ======================================================================
 * FLOAT  (4 bytes on wire, IEEE 754 single precision)
 *
 * Encoded as the bit pattern of an int32_t in big-endian.
 * ====================================================================== */

static inline int __float_encoded_array_size(const float *p, int elements)
{
    (void) p;
    return 4 * elements;
}

static inline int __float_encode_array(void *_buf, int offset, int maxlen,
                                       const float *p, int elements)
{
    /* Use memcpy to bit-cast float→uint32 (avoids strict-aliasing UB) */
    int total_size = 4 * elements;
    if (maxlen < total_size) return -1;
    uint8_t *buf = (uint8_t *)_buf;
    int pos = offset;
    int i;
    for (i = 0; i < elements; i++) {
        uint32_t v;
        memcpy(&v, &p[i], sizeof(v));
        buf[pos++] = (uint8_t)((v >> 24) & 0xff);
        buf[pos++] = (uint8_t)((v >> 16) & 0xff);
        buf[pos++] = (uint8_t)((v >> 8) & 0xff);
        buf[pos++] = (uint8_t)(v & 0xff);
    }
    return total_size;
}

static inline int __float_decode_array(const void *_buf, int offset,
                                       int maxlen, float *p, int elements)
{
    int total_size = 4 * elements;
    if (maxlen < total_size) return -1;
    const uint8_t *buf = (const uint8_t *)_buf;
    int pos = offset;
    int i;
    for (i = 0; i < elements; i++) {
        uint32_t v = (((uint32_t)buf[pos] << 24) +
                      ((uint32_t)buf[pos + 1] << 16) +
                      ((uint32_t)buf[pos + 2] << 8) +
                      (uint32_t)buf[pos + 3]);
        memcpy(&p[i], &v, sizeof(v));
        pos += 4;
    }
    return total_size;
}

/* ======================================================================
 * DOUBLE  (always 8 bytes on wire, IEEE 754 double precision)
 *
 * On platforms where sizeof(double)==8 (x86, ARM Cortex-M4F, etc.) this
 * is a straight bit-cast to int64_t, identical to upstream LCM.
 *
 * On AVR where sizeof(double)==4 (double is aliased to float), we
 * promote float→double on encode and truncate double→float on decode
 * so the wire format stays LCM-compatible.  Precision beyond float32
 * is lost, which is fine for Arduino sensor data.
 * ====================================================================== */

static inline int __double_encoded_array_size(const double *p, int elements)
{
    (void) p;
    return 8 * elements;  /* always 8 bytes on the wire */
}

#if defined(__AVR__)
/* ------- AVR: double is 4 bytes, must promote to 8 on the wire ------- */

/*
 * Minimal float32 → float64 bit conversion (no libm required).
 * Handles: normals, zero, infinity.  Denorms flush to zero.
 */
static inline uint64_t _dimos_f32_to_f64_bits(float f)
{
    union { float f; uint32_t u; } src;
    src.f = f;
    uint32_t s = src.u >> 31;
    uint32_t e = (src.u >> 23) & 0xff;
    uint32_t m = src.u & 0x7fffff;

    uint64_t out;
    if (e == 0) {
        /* zero or denorm → encode as zero */
        out = (uint64_t)s << 63;
    } else if (e == 0xff) {
        /* inf / nan */
        out = ((uint64_t)s << 63) | ((uint64_t)0x7ff << 52) |
              ((uint64_t)m << 29);
    } else {
        /* normal: rebias exponent  127 → 1023 */
        uint64_t e64 = (uint64_t)(e - 127 + 1023);
        out = ((uint64_t)s << 63) | (e64 << 52) | ((uint64_t)m << 29);
    }
    return out;
}

static inline float _dimos_f64_bits_to_f32(uint64_t bits)
{
    uint32_t s = (uint32_t)(bits >> 63);
    uint32_t e64 = (uint32_t)((bits >> 52) & 0x7ff);
    uint32_t m = (uint32_t)((bits >> 29) & 0x7fffff);

    uint32_t out;
    if (e64 == 0) {
        out = s << 31;  /* zero */
    } else if (e64 == 0x7ff) {
        out = (s << 31) | 0x7f800000 | m;  /* inf / nan */
    } else {
        int32_t e32 = (int32_t)e64 - 1023 + 127;
        if (e32 <= 0) {
            out = s << 31;  /* underflow → zero */
        } else if (e32 >= 0xff) {
            out = (s << 31) | 0x7f800000;  /* overflow → inf */
        } else {
            out = (s << 31) | ((uint32_t)e32 << 23) | m;
        }
    }
    union { uint32_t u; float f; } dst;
    dst.u = out;
    return dst.f;
}

static inline int __double_encode_array(void *_buf, int offset, int maxlen,
                                        const double *p, int elements)
{
    int total_size = 8 * elements;
    if (maxlen < total_size)
        return -1;

    int i;
    int64_t tmp;
    for (i = 0; i < elements; i++) {
        tmp = (int64_t)_dimos_f32_to_f64_bits((float)p[i]);
        int ret = __int64_t_encode_array(_buf, offset + i * 8,
                                         maxlen - i * 8, &tmp, 1);
        if (ret < 0) return ret;
    }
    return total_size;
}

static inline int __double_decode_array(const void *_buf, int offset,
                                        int maxlen, double *p, int elements)
{
    int total_size = 8 * elements;
    if (maxlen < total_size)
        return -1;

    int i;
    int64_t tmp;
    for (i = 0; i < elements; i++) {
        int ret = __int64_t_decode_array(_buf, offset + i * 8,
                                         maxlen - i * 8, &tmp, 1);
        if (ret < 0) return ret;
        p[i] = (double)_dimos_f64_bits_to_f32((uint64_t)tmp);
    }
    return total_size;
}

#else
/* ------- Normal platforms: sizeof(double)==8, same as upstream LCM ---- */

static inline int __double_encode_array(void *_buf, int offset, int maxlen,
                                        const double *p, int elements)
{
    /* Use memcpy to bit-cast double→uint64 (avoids strict-aliasing UB) */
    int total_size = 8 * elements;
    if (maxlen < total_size) return -1;
    uint8_t *buf = (uint8_t *)_buf;
    int pos = offset;
    int i;
    for (i = 0; i < elements; i++) {
        uint64_t v;
        memcpy(&v, &p[i], sizeof(v));
        buf[pos++] = (uint8_t)((v >> 56) & 0xff);
        buf[pos++] = (uint8_t)((v >> 48) & 0xff);
        buf[pos++] = (uint8_t)((v >> 40) & 0xff);
        buf[pos++] = (uint8_t)((v >> 32) & 0xff);
        buf[pos++] = (uint8_t)((v >> 24) & 0xff);
        buf[pos++] = (uint8_t)((v >> 16) & 0xff);
        buf[pos++] = (uint8_t)((v >> 8) & 0xff);
        buf[pos++] = (uint8_t)(v & 0xff);
    }
    return total_size;
}

static inline int __double_decode_array(const void *_buf, int offset,
                                        int maxlen, double *p, int elements)
{
    int total_size = 8 * elements;
    if (maxlen < total_size) return -1;
    const uint8_t *buf = (const uint8_t *)_buf;
    int pos = offset;
    int i;
    for (i = 0; i < elements; i++) {
        uint64_t a = (((uint32_t)buf[pos] << 24) +
                      ((uint32_t)buf[pos + 1] << 16) +
                      ((uint32_t)buf[pos + 2] << 8) +
                      (uint32_t)buf[pos + 3]);
        pos += 4;
        uint64_t b = (((uint32_t)buf[pos] << 24) +
                      ((uint32_t)buf[pos + 1] << 16) +
                      ((uint32_t)buf[pos + 2] << 8) +
                      (uint32_t)buf[pos + 3]);
        pos += 4;
        uint64_t v = (a << 32) + (b & 0xffffffff);
        memcpy(&p[i], &v, sizeof(v));
    }
    return total_size;
}

#endif /* __AVR__ double size check */

/* ======================================================================
 * Compile-time guards: refuse variable-length types
 * ====================================================================== */

#ifdef __AVR__
/*
 * On AVR: refuse string/variable-length types at compile time.
 */
#define __string_encode_array(...)  \
    DIMOS_STATIC_ASSERT_FAIL("LCM string types are not supported on Arduino")
#define __string_decode_array(...)  \
    DIMOS_STATIC_ASSERT_FAIL("LCM string types are not supported on Arduino")
#define __string_encoded_array_size(...)  \
    DIMOS_STATIC_ASSERT_FAIL("LCM string types are not supported on Arduino")
#define __string_decode_array_cleanup(...)  \
    DIMOS_STATIC_ASSERT_FAIL("LCM string types are not supported on Arduino")
#define __string_clone_array(...)  \
    DIMOS_STATIC_ASSERT_FAIL("LCM string types are not supported on Arduino")
#else
/*
 * On x86_64/ARM: provide string and malloc helpers so LCM C++ headers
 * compile.  These are only used by types with string fields (Header, etc.)
 * which the Arduino side doesn't support.
 */
#include <stdlib.h>

#define __string_hash_recursive(p) 0

static inline int __string_decode_array_cleanup(char **s, int elements)
{
    int i;
    for (i = 0; i < elements; i++)
        free(s[i]);
    return 0;
}

static inline int __string_encoded_array_size(char *const *s, int elements)
{
    int size = 0, i;
    for (i = 0; i < elements; i++)
        size += 4 + (int)strlen(s[i]) + 1;
    return size;
}

static inline int __string_encoded_size(char *const *s)
{
    return (int)sizeof(int64_t) + __string_encoded_array_size(s, 1);
}

static inline int __string_encode_array(void *_buf, int offset, int maxlen,
                                        char *const *p, int elements)
{
    int pos = 0, thislen, i;
    for (i = 0; i < elements; i++) {
        int32_t length = (int32_t)strlen(p[i]) + 1;
        thislen = __int32_t_encode_array(_buf, offset + pos, maxlen - pos, &length, 1);
        if (thislen < 0) return thislen;
        pos += thislen;
        thislen = __int8_t_encode_array(_buf, offset + pos, maxlen - pos, (int8_t *)p[i], length);
        if (thislen < 0) return thislen;
        pos += thislen;
    }
    return pos;
}

static inline int __string_decode_array(const void *_buf, int offset, int maxlen,
                                        char **p, int elements)
{
    int pos = 0, thislen, i;
    for (i = 0; i < elements; i++) {
        int32_t length;
        thislen = __int32_t_decode_array(_buf, offset + pos, maxlen - pos, &length, 1);
        if (thislen < 0) return thislen;
        pos += thislen;
        p[i] = (char *)malloc(length);
        thislen = __int8_t_decode_array(_buf, offset + pos, maxlen - pos, (int8_t *)p[i], length);
        if (thislen < 0) return thislen;
        pos += thislen;
    }
    return pos;
}

static inline int __string_clone_array(char *const *p, char **q, int elements)
{
    int i;
    for (i = 0; i < elements; i++) {
        size_t len = strlen(p[i]) + 1;
        q[i] = (char *)malloc(len);
        memcpy(q[i], p[i], len);
    }
    return 0;
}

static inline void *lcm_malloc(size_t sz)
{
    if (sz) return malloc(sz);
    return NULL;
}
#endif /* __AVR__ */

/* No-ops for decode cleanup (nothing to free without malloc) */
#define __byte_decode_array_cleanup(p, sz)    do {} while(0)
#define __int8_t_decode_array_cleanup(p, sz)  do {} while(0)
#define __boolean_decode_array_cleanup(p, sz) do {} while(0)
#define __int16_t_decode_array_cleanup(p, sz) do {} while(0)
#define __int32_t_decode_array_cleanup(p, sz) do {} while(0)
#define __int64_t_decode_array_cleanup(p, sz) do {} while(0)
#define __float_decode_array_cleanup(p, sz)   do {} while(0)
#define __double_decode_array_cleanup(p, sz)  do {} while(0)

/* Encoded size macros (hash + field).  Used by LCM-generated code. */
#define byte_encoded_size(p)    ((int)(sizeof(int64_t) + sizeof(uint8_t)))
#define int8_t_encoded_size(p)  ((int)(sizeof(int64_t) + sizeof(int8_t)))
#define boolean_encoded_size    int8_t_encoded_size
#define int16_t_encoded_size(p) ((int)(sizeof(int64_t) + sizeof(int16_t)))
#define int32_t_encoded_size(p) ((int)(sizeof(int64_t) + sizeof(int32_t)))
#define int64_t_encoded_size(p) ((int)(sizeof(int64_t) + sizeof(int64_t)))
#define float_encoded_size(p)   ((int)(sizeof(int64_t) + sizeof(float)))
#define double_encoded_size(p)  ((int)(sizeof(int64_t) + 8))

/* Hash macros (no-ops, bridge handles the fingerprint) */
#define __byte_hash_recursive(p)    0
#define __int8_t_hash_recursive(p)  0
#define __boolean_hash_recursive(p) 0
#define __int16_t_hash_recursive(p) 0
#define __int32_t_hash_recursive(p) 0
#define __int64_t_hash_recursive(p) 0
#define __float_hash_recursive(p)   0
#define __double_hash_recursive(p)  0

/*
 * Introspection types.  Used by LCM C++ generated code.
 * On AVR these are dead code but don't increase binary size
 * (static inline / unused struct definitions are stripped).
 */
typedef enum {
    LCM_FIELD_INT8_T,
    LCM_FIELD_INT16_T,
    LCM_FIELD_INT32_T,
    LCM_FIELD_INT64_T,
    LCM_FIELD_BYTE,
    LCM_FIELD_FLOAT,
    LCM_FIELD_DOUBLE,
    LCM_FIELD_STRING,
    LCM_FIELD_BOOLEAN,
    LCM_FIELD_USER_TYPE
} lcm_field_type_t;

#define LCM_TYPE_FIELD_MAX_DIM 50

typedef struct _lcm_field_t lcm_field_t;
struct _lcm_field_t {
    const char *name;
    lcm_field_type_t type;
    const char *typestr;
    int num_dim;
    int32_t dim_size[LCM_TYPE_FIELD_MAX_DIM];
    int8_t dim_is_variable[LCM_TYPE_FIELD_MAX_DIM];
    void *data;
};

typedef int (*lcm_encode_t)(void *buf, int offset, int maxlen, const void *p);
typedef int (*lcm_decode_t)(const void *buf, int offset, int maxlen, void *p);
typedef int (*lcm_decode_cleanup_t)(void *p);
typedef int (*lcm_encoded_size_t)(const void *p);
typedef int (*lcm_struct_size_t)(void);
typedef int (*lcm_num_fields_t)(void);
typedef int (*lcm_get_field_t)(const void *p, int i, lcm_field_t *f);
typedef int64_t (*lcm_get_hash_t)(void);

typedef struct _lcm_type_info_t lcm_type_info_t;
struct _lcm_type_info_t {
    lcm_encode_t encode;
    lcm_decode_t decode;
    lcm_decode_cleanup_t decode_cleanup;
    lcm_encoded_size_t encoded_size;
    lcm_struct_size_t struct_size;
    lcm_num_fields_t num_fields;
    lcm_get_field_t get_field;
    lcm_get_hash_t get_hash;
};

#ifdef __cplusplus
}
#if defined(__GNUC__) && defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
#endif

#endif /* _LCM_LIB_INLINE_H */
