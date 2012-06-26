/**
 *  \file     filter.h
 *  \brief    1st and 2nd order filter implementation
 *  \author   Jean Baptiste Lanfrey <jean-baptiste.lanfrey@parrot.com>
 *  \version  1.0
 */

#ifndef _FILTER_H_
#define _FILTER_H_

#include <VP_Os/vp_os_types.h>


#define NB_FIRST_ORDER 	              1
#define NB_SECOND_ORDER               2
#define NB_THIRD_ORDER 	              3
#define NB_FOURTH_ORDER               4
#define NB_SIXTH_ORDER                6
#define NB_SEVENTH_ORDER              7
#define DEFAULT_DELAY_STEP_TIME_DELAY TWENTY_STEP_TIME_DELAY
#define FORTY_STEP_TIME_DELAY         40
#define THIRTY_TWO_STEP_TIME_DELAY    32
#define TWENTY_STEP_TIME_DELAY        20
#define filterSamples                 511

///////////////////////////////////////////////
// STRUCTURES


/**
 * \struct _first_order_filter_
 * \brief  First order filter states using Matlab notation.
 */
typedef struct {
  float32_t old_outputs[NB_FIRST_ORDER];  //< filter output history
  float32_t old_inputs [NB_FIRST_ORDER];  //< filter input history
} first_order_filter_t;


/**
 * \struct _second_order_filter_
 * \brief  Second order filter states using Matlab notation.
 */
typedef struct {
  float32_t old_outputs[NB_SECOND_ORDER]; //< filter output history
  float32_t old_inputs [NB_SECOND_ORDER]; //< filter input history
} second_order_filter_t;


/**
 * \struct _second_order_filter_64
 * \brief  Second order filter states using Matlab notation.
 */
typedef struct {
  float64_t old_outputs[NB_SECOND_ORDER]; //< filter output history
  float64_t old_inputs [NB_SECOND_ORDER]; //< filter input history
} second_order_filter_64_t;


/**
 * \struct _third_order_filter_
 * \brief  Third order filter states using Matlab notation.
 */
typedef struct {
  float32_t old_outputs[NB_THIRD_ORDER];  //< filter output history
  float32_t old_inputs [NB_THIRD_ORDER];  //< filter input history
} third_order_filter_t;


/**
 * \struct _fourth_order_filter_
 * \brief  Fourth order filter states using Matlab notation.
 */
typedef struct {
  float32_t old_outputs[NB_FOURTH_ORDER]; //< filter output history
  float32_t old_inputs [NB_FOURTH_ORDER]; //< filter input history
} fourth_order_filter_t;

typedef struct {
  float32_t old_outputs[NB_SIXTH_ORDER];  //< filter output history
  float32_t old_inputs [NB_SIXTH_ORDER];  //< filter input history
} sixth_order_filter_t;

typedef struct {
  float32_t old_outputs[NB_SEVENTH_ORDER];  //< filter output history
  float32_t old_inputs [NB_SEVENTH_ORDER];  //< filter input history
} seventh_order_filter_t;


typedef struct _deriv_param_t {
  float32_t kd;
  float32_t td;
  float32_t te;

  float32_t internal_state;
  float32_t old_input;
} deriv_param_t;

/**
 * \struct _delay_
 * \brief  m sampling step time delay.
 */
typedef struct {
  float32_t old_inputs[FORTY_STEP_TIME_DELAY];  //< input history
} delay_t;

///////////////////////////////////////////////
// FUNCTIONS


/**
 * \fn      Digital filter initialization.
 * \brief   This function is used to initialize previous values in digital filter.
 * \param   Filter order.
 * \param   address of previous inputs list.
 * \param   initial input value.
 * \param   address of previous outputs list.
 * \param   initial output value.
 * \return  void.
 *
 * \section History
 *
 * \par date: 2007-06-25  author: <florian.pantaleao.ext\@parrot.com> <jean-baptiste.lanfrey\@parrot.com>
 *  - first version
 */
void filter_init(uint32_t n, float32_t *old_input, float32_t initial_input, float32_t *old_output, float32_t initial_output);



/**
 * \fn      Filter a value.
 * \brief   This function uses the same notation as Matlab except that array indices start at 0
 * \brief   a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb) - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
 * \brief   This function automatically shifts old inputs and outputs.
 * \param   Filter order
 * \param   address of B coefficients list.
 * \param   address of A coefficients list.
 * \param   input value.
 * \param   address of previous inputs list.
 * \param   address of previous outputs list.
 * \return  filtered value.
 *
 * \section History
 *
 * \par date: 2007-06-25  author: <florian.pantaleao.ext\@parrot.com> <jean-baptiste.lanfrey\@parrot.com>
 *  - first version
 */
float32_t filter(uint32_t n, const float32_t *b, const float32_t *a, float32_t input, float32_t *old_input, float32_t *old_output);

/**
 * \fn      Derivative filter
 * \brief   Y/U = kd;p/(1+Td.p)
 * \param   Kd, Td, Te, previous values (state and input)
 * \return  filtered value.
 *
 * \section History
 *
 * \par date: 2007-10-15  author: <jean-baptiste.lanfrey\@parrot.com>
 *  - first version
 */
float32_t deriv(deriv_param_t *param, float32_t input);

/**
 * \fn      Digital delay initialization
 * \brief   This function is used to initialize previous inputs of the digital delay.
 * \param   m number of sampling step time delay.
 * \return  void.
 *
 * \section History
 *
 * \par date: 2008-5-13  author: <yannick.foloppe.ext\@parrot.com>
 *  - first version
 */
void delay_init(uint32_t m, float32_t *old_input, float32_t initial_input);

/**
 * \fn      Delay a value
 * \brief   y(k)=u(k-m).
 * \param   m number of sampling step time delay, previous values (state and input).
 * \return  delayed input.
 *
 * \section History
 *
 * \par date: 2008-5-13  author: <yannick.foloppe.ext\@parrot.com>
 *  - first version
 */
float delay(uint32_t m, float32_t input, float32_t *old_input);

// rate limiter
// rate_max is the highest rate allowed in one sample time
// warning : rate_max must be positive
/**
 * \fn      rate limiter
 * \brief  limit the rate of an input.
 * \param   rate_max is the highest rate allowed in one sample time. Warning : rate_max must be positive
 * \return  rate limiter output.
 *
 * \section History
 *
 * \par date: 2008-7-24 author: <yannick.foloppe.ext\@parrot.com>
 *  - first version
 */
float32_t rate_limiter(float32_t input, float32_t old_output, float32_t rate_max);
int32_t digitalsmooth(int32_t rawIn, int32_t *sensSmoothArray);
void unwrapToPi(float32_t* former_wrapped_angle, float32_t* former_unwrapped_angle, float32_t variation);
void wrapToPi(float32_t* input, float32_t* output);

void filter64_init(uint32_t n, float64_t *old_input, float64_t initial_input, float64_t *old_output, float64_t initial_output);
float64_t filter64(uint32_t n, const float64_t *b, const float64_t *a, float64_t input, float64_t *old_input, float64_t *old_output);

#endif // FILTER
