// ----------------------------------------------
//
//  Author : <sylvain.gaeremynck@parrot.fr>
//  Date   : 15/01/2007
//
//  Parrot Video SDK : Examples/common
//
// ----------------------------------------------

#ifndef _COM_SERVER_H_
#define _COM_SERVER_H_

#include <VP_Os/vp_os_types.h>
#include <Com/com.h>

C_RESULT init_com_server(void);
C_RESULT run_com_server(COM_PROTOCOL protocol);
C_RESULT shutdown_com_server(void);

C_RESULT read_server(int8_t* buffer, int32_t* size);
C_RESULT write_server(const int8_t* buffer, int32_t* size);

#endif // _COM_SERVER_H_
