/**
 * @file vp_os_delay.c
 * @author aurelien.morelle@parrot.com
 * @date 2007/03/04
 */

#include "VP_Os/vp_os_delay.h"

#ifdef ARM7
# define HW_CPU_CLOCK                HW_CPU_CLOCK_ARM7
#endif // ! ARM7

#ifdef ARM9
# define HW_CPU_CLOCK                HW_CPU_CLOCK_ARM9
#endif // ! ARM9

void vp_os_delay(uint32_t ms)
{
}

void vp_os_delay_us(uint32_t us)
{
}

