#include <VP_Os/vp_os_malloc.h>

#include <config.h>
#include <ardrone_tool/Com/config_com.h>

#ifndef _WIN32

vp_com_t* serial_com(void)
{
  static vp_com_t com = {
    VP_COM_SERIAL,
    FALSE,
    0,
#ifdef _WIN32
	0,
#else
	PTHREAD_MUTEX_INITIALIZER,
#endif
    NULL,
    NULL,
    0,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
  };

  return &com;
}

vp_com_config_t* serial_config_0(void)
{
  static vp_com_serial_config_t config = {
    SERIAL_LINK_0,
    VP_COM_BAUDRATE_115200,
    SL0_BAUDRATE,
    1,
    0,
    0
  };

  return (vp_com_config_t*) &config;
}

vp_com_config_t* serial_config_1(void)
{
  static vp_com_serial_config_t config = {
    SERIAL_LINK_1,
    VP_COM_BAUDRATE_115200,
    SL1_BAUDRATE,
    1,
    0,
    1
  };

  return (vp_com_config_t*) &config;
}

vp_com_config_t* serial_config_2(void)
{
  static vp_com_serial_config_t config = {
    SERIAL_LINK_2,
    VP_COM_BAUDRATE_115200,
    SL2_BAUDRATE,
    1,
    0,
    0
  };

  return (vp_com_config_t*) &config;
}

vp_com_connection_t*  serial_connection_0(void)
{
  static vp_com_connection_t connection = { 0 };

  return &connection;
}

vp_com_connection_t*  serial_connection_1(void)
{
  static vp_com_connection_t connection = { 0 };

  return &connection;
}

vp_com_connection_t*  serial_connection_2(void)
{
  static vp_com_connection_t connection = { 0 };

  return &connection;
}

void serial_config_socket(vp_com_socket_t* socket, VP_COM_SOCKET_TYPE type)
{
  vp_os_memset(socket, 0, sizeof(vp_com_socket_t));

  socket->type = type;
}

#endif
