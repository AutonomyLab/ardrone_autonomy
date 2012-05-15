/**
 *  \file     vp_com_interface.h
 *  \brief    Com Api for video sdk. Private definitions.
 *  \brief    Provide default behavior of a vp products
 */

#ifndef _VP_COM_INTERFACE_H_
#define _VP_COM_INTERFACE_H_

//#include <parrotos/parrotos.h>
#include <VP_Com/vp_com.h>
#include <VP_Os/vp_os_types.h>

extern char vp_com_pin_code[VP_COM_MAX_PINCODE_SIZE];

void vp_com_parse_key(const char* passkey);
/**
 * \typedef Prototype of an EXTI handler
 */
typedef unsigned int MESSAGE;
typedef unsigned int PARAM;
typedef C_RESULT (*vp_com_interface_handler_cb)(MESSAGE message, PARAM param);

extern vp_com_interface_handler_cb prod_interface_handler;

/** start module EXTI
 * \par Brief :
 * \code
 *  char tab[] = {
 *                 "bluetooth hardware",
 *                 "bluetooth uart",
 *                 "flash configuration",
 *                 "blues Exti"
 *               };
 *
 *  for( x = 0 ; x <= endTab; x++)
 *   {
 *     launch tab[x] function specfied by the application
 *   }
 * \endcode
 *
 * \section History
 *
 * \par June, 27 2007 - <thomas.landais\@parrot.com>
 *  - add flash configuration
 *
 * \par March, 16 2007 - <sylvain.gaeremynck\@parrot.com>
 *  - first release
 *
*/
C_RESULT vp_com_start_exti(void) __attribute__((weak));

#ifdef USE_WIFI
C_RESULT vp_com_wait_set_ssid(void);
C_RESULT vp_com_wait_scan_complete(void);
#endif

#endif // _VP_COM_INTERFACE_H_
