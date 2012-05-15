#include <VP_Os/vp_os_delay.h>

#undef TRUE
#undef FALSE
#include "parrotOS_thread.h"

void vp_os_delay(uint32_t ms)
{
  sup_thread_delay(ms);
}

void vp_os_delay_us(uint32_t us)
{
  sup_thread_udelay(us);
}

