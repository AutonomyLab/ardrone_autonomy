#include "common.h"

#include <Com/com.h>
#include <Api/error.h>

#include "com_client.h"

static com_config_t     config;
static com_socket_t     clt;
static com_connection_t connection;
static Read             read;
static Write            write;

C_RESULT init_com_client(void)
{
  config.connection       = COM_BLUETOOTH;
  config.localAdapterName = DEVICENAME;
  config.localIpAddress   = CLIENTHOST;
  config.localIpSubmask   = SUBMASK;

  if(FAILED(com_init(&config)))
    return C_FAIL;

  return C_OK;
}

C_RESULT run_com_client(COM_PROTOCOL protocol)
{
  com_strToAddress(BTADDR_SERVER,&connection.address);

  com_passKey(PIN_CODE);
  if(FAILED(com_connect(&connection,1)))
    return C_FAIL;

  clt.socket    = COM_CLIENT;
  clt.protocol  = protocol;

  if(protocol == COM_RFCOMM)
  {
    clt.scn = BTADDR_SCN;
  }
  else if(protocol == COM_BNEP)
  {
    clt.port        = BTADDR_PORT;
    clt.serverHost  = SERVERHOST;
  }
  else
    return C_FAIL;

  if(FAILED(com_open(&clt,&read,&write)))
    return C_FAIL;

  return C_OK;
}

C_RESULT shutdown_com_client(void)
{
  com_close(&clt);

  com_disconnect();
  com_shutdown();

  return C_OK;
}

C_RESULT read_client(int8_t* buffer, int32_t* size)
{
  return read(&clt,buffer,size);
}

C_RESULT write_client(const int8_t* buffer, int32_t* size)
{
  return write(&clt,buffer,size);
}
