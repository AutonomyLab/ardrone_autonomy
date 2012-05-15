// ----------------------------------------------
//
//  Author : <sylvain.gaeremynck\@parrot.fr>
//  Date   : 15/01/2007
//
//  Parrot Video SDK : Examples/common
//
// ---------------------------------------------- 

#include "common.h"

#include <Com/com.h>
#include <Api/error.h>
#include <VP_Os/vp_os_print.h>

#include "com_server.h"

static com_config_t config;
static com_socket_t srv;
static com_socket_t clt;
static Read         read;
static Write        write;

C_RESULT init_com_server(void)
{
  config.connection       = COM_BLUETOOTH;
  config.localAdapterName = DEVICENAME;
  config.localIpAddress   = SERVERHOST;
  config.localIpSubmask   = SUBMASK;

  if(FAILED(com_init(&config)))
    return C_FAIL;

  return C_OK;
}

C_RESULT run_com_server(COM_PROTOCOL protocol)
{
  srv.socket    = COM_SERVER;
  srv.protocol  = protocol;

  if(protocol == COM_RFCOMM)
  {
    srv.scn = BTADDR_SCN;
  }
  else if(protocol == COM_BNEP)
  {
    srv.port        = BTADDR_PORT;
    srv.serverHost  = SERVERHOST;
  }
  else
    return C_FAIL;

  if(FAILED(com_open(&srv,&read,&write)))
    return C_FAIL;

  com_passKey(PIN_CODE);
  if(FAILED(com_accept(&srv,&clt)))
    return C_FAIL;

  return C_OK;
}

C_RESULT shutdown_com_server(void)
{
  com_close(&clt);
  com_close(&srv);

  com_shutdown();

  return C_OK;
}

C_RESULT read_server(int8_t* buffer, int32_t* size)
{
  return read(&clt,buffer,size);
}

C_RESULT write_server(const int8_t* buffer, int32_t* size)
{
  return write(&clt,buffer,size);
}
