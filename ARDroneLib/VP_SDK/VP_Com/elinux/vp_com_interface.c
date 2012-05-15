/**
 *  \file     vp_com_interface.h
 *  \brief    Com Api for video sdk. Private definitions.
 *  \brief    Provide default behavior of a vp products
 */

#include <VP_Os/vp_os_delay.h>

#include <VP_Com/vp_com.h>
#include <VP_Com/vp_com_error.h>
#include "vp_com_interface.h"

C_RESULT vp_com_start_exti(void)
{
  return VP_COM_OK;
}

#ifdef USE_WIFI

C_RESULT vp_com_wait_set_ssid(void)
{
  vp_os_delay(500);

  return VP_COM_OK;
}

C_RESULT vp_com_wait_scan_complete(void)
{
  vp_os_delay(500);

  return VP_COM_OK;
}

#endif // < USE_WIFI
