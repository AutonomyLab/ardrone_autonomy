// ----------------------------------------------
//
//  Author : <sylvain.gaeremynck@parrot.fr>
//  Date   : 03/01/2007
//
//  Parrot Video SDK : Examples/common
//
// ----------------------------------------------

#ifndef _COMMON_H_
#define _COMMON_H_

#define BTADDR_SERVER   /* "AE:11:A5:22:DE:DE" */ "00:0A:3A:6B:DB:B9"
#define BTADDR_SCN      9
#define BTADDR_PORT     5555

#define SERVERHOST      "192.168.2.2"
#define CLIENTHOST      "192.168.2.1"
#define LOCALHOST       "192.168.2.2"
#define SUBMASK         "255.255.255.0"

#define PIN_CODE        "1234"

#ifdef _WIN32
#define DEVICENAME      "Bluetooth Network"
#endif

#ifdef __linux__
#define DEVICENAME      "hci0"
#endif

#include <VP_Os/vp_os_types.h>

void deviceinquiry_df(bdaddr_t* address,const char* name);
void adapterinquiry_df(const char* name);

#endif // _COMMON_H_
