/**
 * @file delay.c
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/15
 */

#include <windows.h>

#include "VP_Os/vp_os_delay.h"


void vp_os_delay(uint32_t ms)
{
  Sleep((DWORD)ms);
}

void vp_os_delay_us(uint32_t us)
{
  vp_os_delay(us/1000);
}

