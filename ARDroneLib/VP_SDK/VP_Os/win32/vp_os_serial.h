/**
 *  \brief    Baudrate definitions.
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.com>
 *  \version  1.0
 *  \date     15/02/2008
 */

#ifndef _VP_OS_SERIAL_H_
#define _VP_OS_SERIAL_H_

#include <windows.h>

typedef enum _vp_com_baudrates_
{
  VP_COM_BAUDRATE_110 = CBR_110,
  VP_COM_BAUDRATE_300 = CBR_300,
  VP_COM_BAUDRATE_600 = CBR_600,
  VP_COM_BAUDRATE_1200 = CBR_1200,
  VP_COM_BAUDRATE_2400 = CBR_2400,
  VP_COM_BAUDRATE_4800 = CBR_4800,
  VP_COM_BAUDRATE_9600 = CBR_9600,
  VP_COM_BAUDRATE_14400 = CBR_14400,
  VP_COM_BAUDRATE_19200 = CBR_19200,
  VP_COM_BAUDRATE_38400 = CBR_38400,
  VP_COM_BAUDRATE_57600 = CBR_57600,
  VP_COM_BAUDRATE_115200 = CBR_115200,
  VP_COM_BAUDRATE_128000 = CBR_128000,
  VP_COM_BAUDRATE_256000 = CBR_256000,
  VP_COM_BAUDRATE_460800 = CBR_256000, // TODO Added only for 
} vp_com_baudrate_t;

#endif // _VP_OS_SERIAL_H_
