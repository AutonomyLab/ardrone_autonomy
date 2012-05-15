#include <VP_Com/vp_com_error.h>
#include <VP_Com/vp_com_socket.h>

#include "vp_com_config_itf.h"

#include <VP_Os/vp_os_malloc.h>

#include <netdb.h>
#include <sys/socket.h>

#include <net/if.h>
#include <netinet/in.h>

#include <stdio.h>


C_RESULT vp_com_wired_init(void)
{
  return VP_COM_OK;
}

C_RESULT vp_com_wired_shutdown(void)
{
  return VP_COM_OK;
}

C_RESULT vp_com_wired_network_adapter_lookup(vp_com_network_adapter_lookup_t callback)
{
  return VP_COM_ERROR;
}

C_RESULT vp_com_wired_inquire(const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout)
{
  return VP_COM_ERROR;
}

C_RESULT vp_com_wired_local_config(vp_com_wired_config_t* cfg)
{
  return vp_com_config_itf( cfg->itfName, cfg->localHost, cfg->broadcast, cfg->netmask );
}

C_RESULT vp_com_wired_connect(vp_com_t* vp_com, vp_com_wired_connection_t* connection, int32_t numAttempts)
{
  return VP_COM_OK;
}

C_RESULT vp_com_wired_disconnect(vp_com_wired_config_t* config, vp_com_wired_connection_t* connection)
{
  return VP_COM_OK;
}

C_RESULT vp_com_wired_wait_connections(vp_com_wired_connection_t** c, vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength)
{
  return vp_com_wait_socket(server, client, queueLength);
}

C_RESULT vp_com_wired_open(vp_com_wired_config_t* config, vp_com_wired_connection_t* connection, vp_com_socket_t* sck, Read* read, Write* write)
{
  return vp_com_open_socket(sck, read, write);
}

C_RESULT vp_com_wired_close(vp_com_socket_t* socket)
{
  return vp_com_close_socket(socket);
}
