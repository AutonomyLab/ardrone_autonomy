/**
 * @file delay.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#ifndef _DELAY_INCLUDE_OS_
#define _DELAY_INCLUDE_OS_


#include <VP_Os/vp_os_types.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Waits until delay is elapsed.
 *
 * @param ms Delay in milliseconds
 */
void
vp_os_delay(uint32_t ms);

/**
 * Waits until delay is elapsed.
 *
 * @param us Delay in microseconds
 */
void
vp_os_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif


#endif // ! _DELAY_INCLUDE_OS_

