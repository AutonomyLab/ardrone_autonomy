/**
 *  \file     maths.h
 *  \brief    Maths library used by ARDrone
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.com>
 *  \version  1.0
 */

#ifndef _MATHS_H_
#define _MATHS_H_

#include <VP_Os/vp_os_types.h>

#ifndef __USE_GNU
#define __USE_GNU
#endif // __USE_GNU

#include <math.h>
#include <float.h>

///////////////////////////////////////////////
// DEFINES

#define RAD_TO_MDEG (57295.779513f)
#define RAD_TO_DEG  (57.295779513f)
#define MDEG_TO_RAD (1.745329252e-05f)
#define DEG_TO_RAD  (1.745329252e-02f)
#define PI          (3.1415927f)
#define GRAVITY     (9.81f)

#ifndef min
#define min(a, b) (a) < (b) ? (a) : (b)
#endif

#ifndef max
#define max(a, b) (a) < (b) ? (b) : (a)
#endif

typedef struct _screen_point_t {
  int32_t x;
  int32_t y;
} screen_point_t;

typedef union _float_or_int_t {
  float32_t f;
  int32_t   i;
} float_or_int_t;

extern float32_t f_epsilon;

bool_t f_is_zero( float32_t f );
float32_t f_zero( float32_t f );

static inline float32_t f_round(float32_t f, int32_t nb_decimal)
{
	float32_t puissance = (float32_t) (10 ^ nb_decimal);
	return ((int)(f * puissance) / puissance);
}

// Returns f's absolute value
static inline float32_t f_abs( float_or_int_t fi )
{
  fi.i &= 0x7FFFFFFF;

  return fi.f;
}

// Inverse f's sign
static inline void f_inv_sign( float_or_int_t* fi )
{
  fi->i ^= 0x80000000;
}

// Polar saturation
void f_polar_sat( float32_t max, float32_t* phi, float32_t* theta);

// Returns clamp value with value's sign
static inline float32_t f_set_clamp( float_or_int_t value, float_or_int_t clamp_value )
{
  clamp_value.i |= value.i & 0x80000000;

  return clamp_value.f;
}

float32_t asin_taylor( float32_t x );
float32_t atan2_taylor( float32_t num, float32_t den );
float32_t exp_taylor( float32_t x );
float32_t secant_taylor( float32_t x );
float32_t cos_taylor( float32_t x );
float32_t sin_taylor( float32_t x );
float32_t pow_taylor( float32_t x , float32_t y );
float32_t time_navdata_in_ms( uint32_t current_time, int32_t dec );

static inline uint32_t iabs(int32_t v) { return ( v < 0 ) ? -v : v; }

#if defined( USE_MINGW32 )
static inline void sincosf(float32_t a, float32_t* out_s, float32_t* out_c)
{
  __asm(
    "flds %2\n"
    "fsincos\n"
    "fstps %1\n"
    "fstps %0"
    :"=m"(*out_s), "=m"(*out_c)
    :"m"(a)
  );
}
#elif defined( _MSC_VER ) || defined( TARGET_OS_IPHONE) || defined(TARGET_IPHONE_SIMULATOR)
static inline void sincosf(float32_t a, float32_t* out_s, float32_t* out_c)
{
	*out_s = sinf(a);
	*out_c = cosf(a);
}
#endif // _MSC_VER || USE_MINGW32

// 8 bits version
uint32_t nb_bits_differents_8(uint32_t p, uint32_t q);

uint32_t nb_bits_differents(uint32_t p, uint32_t q);


#endif // _MATHS_H_
