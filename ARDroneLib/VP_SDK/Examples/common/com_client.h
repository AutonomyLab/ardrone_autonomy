// ----------------------------------------------
//
//  Author : <sylvain.gaeremynck@parrot.fr>
//  Date   : 15/01/2007
//
//  Parrot Video SDK : Examples/common
//
// ----------------------------------------------

#ifndef _COM_CLIENT_H_
#define _COM_CLIENT_H_

#include <VP_Os/vp_os_types.h>
#include <Com/com.h>

C_RESULT init_com_client(void);
C_RESULT run_com_client(COM_PROTOCOL protocol);
C_RESULT shutdown_com_client(void);

C_RESULT read_client(int8_t* buffer, int32_t* size);
C_RESULT write_client(const int8_t* buffer, int32_t* size);

#endif // _COM_CLIENT_H_
