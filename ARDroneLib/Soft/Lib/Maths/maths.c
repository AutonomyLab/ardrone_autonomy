/**
 *  \file     maths.c
 *  \brief    Maths library used by ARDrone
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.com>
 *  \author   Jean-Baptiste Lanfrey <jean-baptiste.lanfrey@parrot.com>
 *  \version  1.0
 */
#include <Maths/maths.h>

float32_t f_epsilon = FLT_EPSILON;

bool_t f_is_zero( float32_t f )
{
  bool_t ret;
  float32_t af;

  ret = FALSE;

  af = (float32_t)fabsf( f );

  if( af < f_epsilon )
    ret = TRUE;

  return ret;
}

float32_t f_zero( float32_t f )
{
  if( f_is_zero(f) )
    return 0.0f;

  return f;
}

float32_t asin_taylor( float32_t x )
{
  // float32_t xsquare = x*x;
  // thrid order
  return (float32_t)(x*(1.0f+x*x*0.16666667f));
  // fifth order
  // return (x*(1.0+xsquare*(1.0/6.0 + 3.0*xsquare/40.0);
}

float32_t atan2_taylor( float32_t num, float32_t den )
{
  float32_t res;

  if ( f_is_zero(den) ) {
    // pi/2
    res = 1.5707963268f;
  }
  else {
    float32_t x = num/den;
    // float32_t xsquare = x*x;
    // thrid order
    res = (float32_t)x*(1.0f-x*x*.33333333f);
    // fifth order
    // res = x*(1.0-xsquare*(1.0/3.0 + xsquare/5.0));
  }
  return (res);
}

// Polar saturation
void f_polar_sat( float32_t max, float32_t* phi, float32_t* theta)
{
  float32_t alpha;
  float32_t c_alpha, s_alpha;
  float32_t phi_in;
  float32_t theta_in;

  phi_in = *phi;
  theta_in = *theta;

  // angle beta du plan du drone avec l'horizontal vaut beta = acosf(cosf(theta_in)*cosf(phi_in))
  // on peut approximer par beta^2 = theta_in^2 + phi_in^2
  if ( theta_in*theta_in + phi_in*phi_in > max*max ) {
    alpha = (float32_t)atan2f(theta_in, phi_in);
    sincosf( alpha, &s_alpha, &c_alpha);
    *phi = max * c_alpha;
    *theta = max * s_alpha;
  }
}

float32_t exp_taylor( float32_t x )
{
  return (float32_t)(1.0f+x+x*x*0.5f);
}

float32_t secant_taylor( float32_t x ) // secant(x) = 1 / cos(x)
{
  return (float32_t)(1.0f+x*x*0.5f);
}

float32_t cos_taylor( float32_t x )
{
  return (float32_t)(1.0f-x*x*0.5f);
}

float32_t sin_taylor( float32_t x )
{
  return (float32_t)(x*(1.0f-x*x*.16666667f));
}

float32_t pow_taylor( float32_t x , float32_t y )
{
	//x^y pour x> et y entier
	float32_t coeff_3=(y*(y-1)*(y-2))/6;
	float32_t coeff_2=(y*(y-1))/2;
	float32_t coeff_1=y;
//	float32_t puissance;

	//if y positif{
	  return (float32_t)(coeff_3*(x-1)*(x-1)*(x-1)+coeff_2*(x-1)*(x-1)+(coeff_1*(x-1))+1);
	//else
	  //puissance=(coeff_3*(x-1)*(x-1)*(x-1)+coeff_2*(x-1)*(x-1)+(coeff_1*(x-1))+1);
	  //return (float32_t) 1/puissance;
	  //}
}
float32_t time_navdata_in_ms(uint32_t current_time, int32_t dec)
{
  //convert time given by get_current_time() in millisecond

  uint32_t time_sec, time_micro_sec;
  float32_t time_milli_sec;

  time_sec = current_time >> dec;

  time_micro_sec = current_time - (time_sec << dec);

  time_milli_sec = (float32_t)(1000.0*time_sec + time_micro_sec/1000.0);

  return(time_milli_sec);
}

#ifndef __ARDRONE_BIT_COUNTING_FUNCTIONS__
#define __ARDRONE_BIT_COUNTING_FUNCTIONS__
static uint32_t MASK_01010101 =  (((unsigned int)(-1))/3); // 01010101010101010101010101010101
static uint32_t MASK_00110011 =  (((unsigned int)(-1))/5); // 00110011001100110011001100110011
static uint32_t MASK_00001111 = (((unsigned int)(-1))/17); // 00001111000011110000111100001111

static inline int bitcountNifty_8(unsigned int n, uint32_t MASK_01010101, uint32_t MASK_00110011, uint32_t MASK_00001111)
{
  n = (n & MASK_01010101) + ((n >> 1) & MASK_01010101) ;
  n = (n & MASK_00110011) + ((n >> 2) & MASK_00110011) ;
  n = (n & MASK_00001111) + ((n >> 4) & MASK_00001111) ;
  return n;
}

static inline int bitcountNifty(unsigned int n, uint32_t MASK_01010101, uint32_t MASK_00110011, uint32_t MASK_00001111)
{
  n = (n & MASK_01010101) + ((n >> 1) & MASK_01010101) ;
  n = (n & MASK_00110011) + ((n >> 2) & MASK_00110011) ;
  n = (n & MASK_00001111) + ((n >> 4) & MASK_00001111) ;
  return n % 255 ;
}

uint32_t nb_bits_differents(uint32_t p, uint32_t q)
{
  return bitcountNifty(p^q, MASK_01010101, MASK_00110011, MASK_00001111);
}

uint32_t nb_bits_differents_8(uint32_t p, uint32_t q)
{
  return bitcountNifty_8(p^q, MASK_01010101, MASK_00110011, MASK_00001111);
}
#endif
