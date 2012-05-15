// Header ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 *  \brief    Com Api for video sdk. Private declarations.
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.com>
 *  \version  1.0
 *  \date     24/07/2007
 */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Include //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include <VP_Com/vp_com.h>
#include <VP_Com/vp_com_error.h>

#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_signal.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_delay.h>

#include <VP_Api/vp_api_error.h>

#include "vp_com_serial.h"

// Static ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static struct termios tio_save;
static vp_os_mutex_t wait_sync_mutex;
static vp_os_mutex_t write_sync_mutex;

// Forward declarations /////////////////////////////////////////////////////////////////////////////////////////////////////////

static C_RESULT vp_com_serial_wait_sync(vp_com_serial_config_t* config, vp_com_socket_t* socket);
static C_RESULT vp_com_serial_write_sync(vp_com_serial_config_t* config, vp_com_socket_t* socket);

static C_RESULT vp_com_serial_read (vp_com_socket_t* socket, int8_t* buffer, int32_t* size);
static C_RESULT vp_com_serial_write(vp_com_socket_t* socket, const int8_t* buffer, int32_t* size);

// Functions declaration ////////////////////////////////////////////////////////////////////////////////////////////////////////

C_RESULT vp_com_serial_init(void)
{
  vp_os_mutex_init(&wait_sync_mutex);
  vp_os_mutex_init(&write_sync_mutex);

  return VP_COM_OK;
}

C_RESULT vp_com_serial_shutdown()
{
  // nothing
  return VP_COM_OK;
}

C_RESULT vp_com_serial_local_config(vp_com_serial_config_t* config)
{
  C_RESULT res = VP_COM_OK;

  // Serial port must be opened before changing config

  return res;
}

C_RESULT vp_com_serial_connect(vp_com_t* vp_com, vp_com_connection_t* connection, int32_t numAttempts)
{
  C_RESULT res = VP_COM_OK;

  res = (connection == NULL) ? VP_COM_PARAMERROR : VP_COM_OK;
  VP_COM_CHECK( res );

  connection->is_up = 1;

  return VP_COM_OK;
}

C_RESULT vp_com_serial_disconnect(vp_com_serial_config_t* config, vp_com_connection_t* connection)
{
  C_RESULT res = (connection == NULL) ? VP_COM_PARAMERROR : VP_COM_OK;
  VP_COM_CHECK( res );

  connection->is_up = 0;

  return VP_COM_OK;
}

C_RESULT vp_com_serial_open(vp_com_serial_config_t* config, vp_com_connection_t* connection, vp_com_socket_t* socket, Read* read, Write* write)
{
  struct termios tio;
  speed_t speed;

  VP_OS_ASSERT(config->blocking == 0 || config->blocking == 1);

  if(config->blocking == 0)
    socket->priv = (void *)open(&config->itfName[0], O_RDWR|O_NOCTTY|O_NONBLOCK);
  else
    socket->priv = (void *)open(&config->itfName[0], O_RDWR|O_NOCTTY);

  if(((int)socket->priv) == -1)
    {
      PRINT("Unable to \"open\" serial device");
      return (VP_COM_ERROR);
    }

  /* get current serial port settings */
  if(tcgetattr((int)socket->priv, &tio) != 0)
    {
      PRINT("Serial device configuration failure (%s)", strerror(errno));
      close((int)socket->priv);
      return (VP_COM_ERROR);
    }

  tio_save = tio;

  if(config->sync)
    {
      /* set serial settings */
      speed = (speed_t)config->initial_baudrate;
      cfsetispeed(&tio, speed);
      cfsetospeed(&tio, speed);
      cfmakeraw(&tio);

      if(tcsetattr((int)socket->priv, TCSANOW, &tio) != 0)
	{
	  PRINT("Serial device configuration failure (%s)", strerror(errno));
	  close((int)socket->priv);
	  return (VP_COM_ERROR);
	}

      if(socket->type == VP_COM_CLIENT)
	{
	  if(FAILED(vp_com_serial_write_sync(config, socket)))
	    return (VP_COM_ERROR);
	  vp_os_delay(VP_COM_SYNC_DELAY);
	}
      else if(socket->type == VP_COM_SERVER)
	{
	  if(FAILED(vp_com_serial_wait_sync(config, socket)))
	    return (VP_COM_ERROR);
	  vp_os_delay(VP_COM_SYNC_DELAY);
	}
    }

  /* set serial settings */
  speed = (speed_t)config->baudrate;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  cfmakeraw(&tio);

  if(tcsetattr((int)socket->priv, TCSANOW, &tio) != 0)
    {
      PRINT("Serial device configuration failure (%s)", strerror(errno));
      close((int)socket->priv);
      return (VP_COM_ERROR);
    }

  if(read) *read = (Read) vp_com_serial_read;
  if(write) *write = (Write) vp_com_serial_write;

  return (VP_COM_OK);
}

C_RESULT vp_com_serial_close(vp_com_socket_t* socket)
{
  if(tcsetattr((int)socket->priv, TCSANOW, &tio_save) != 0)
    {
      PRINT("Serial device configuration failure (%s)", strerror(errno));
      close((int)socket->priv);
      return (VP_COM_ERROR);
    }

  close((int)socket->priv);

  return VP_COM_OK;
}

C_RESULT vp_com_serial_wait_connections(vp_com_connection_t** c, vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength)
{
  vp_os_memcpy(client, server, sizeof(vp_com_socket_t));
  return VP_COM_OK;
}

C_RESULT vp_com_serial_network_adapter_lookup(vp_com_network_adapter_lookup_t callback)
{
  // nothing
  return VP_COM_OK;
}

C_RESULT vp_com_serial_inquire(const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout)
{
  // nothing
  return VP_COM_OK;
}

// Static functions /////////////////////////////////////////////////////////////////////////////////////////////////////////////

static C_RESULT vp_com_serial_read (vp_com_socket_t* socket, int8_t* buffer, int32_t* size)
{
  C_RESULT res;

  *size = read((int)socket->priv, buffer, *size);

  if(*size == -1 && errno != EAGAIN)
  {
    res = VP_COM_ERROR;
  }
  else
  {
    res = VP_COM_OK;

    if(*size == -1)
      *size = 0;
  }

  return res;
}

static C_RESULT vp_com_serial_write(vp_com_socket_t* socket, const int8_t* buffer, int32_t* size)
{
  uint32_t remain = *size, written = 0;

  while(remain > 0)
    {
      written = write((int)socket->priv, buffer+(*size-remain), remain);

      if(written == -1 && errno != EAGAIN)
	return VP_COM_ERROR;
      else if(written == -1)
	written = 0;

      remain -= written;
    }

  return VP_COM_OK;
}

static C_RESULT vp_com_serial_wait_sync(vp_com_serial_config_t* config, vp_com_socket_t* socket)
{
  uint32_t nb;
  uint8_t c;

  vp_os_mutex_lock(&wait_sync_mutex);

  if(!config->sync_done)
    {
      nb = 0;
      do
	{
	  if(-1 == read((int)socket->priv, &c, sizeof(int8_t)))
	    return (FAIL);
	  if(c == (VP_COM_SYNC_STRING)[nb])
	    {
	      nb++;
	    }
	  else
	    {
	      nb = 0;
	    }
	}
      while(nb != sizeof(VP_COM_SYNC_STRING));

      config->sync_done = 1;
    }

  vp_os_mutex_unlock(&wait_sync_mutex);

  return (SUCCESS);
}

static C_RESULT vp_com_serial_write_sync(vp_com_serial_config_t* config, vp_com_socket_t* socket)
{
  uint32_t i;
  uint8_t c;

  vp_os_mutex_lock(&write_sync_mutex);

  if(!config->sync_done)
    {
      for(i = 0 ; i < sizeof(VP_COM_SYNC_STRING) ; i++)
	{
	  c = (VP_COM_SYNC_STRING)[i];
	  if(-1 == write((int)socket->priv, &c, sizeof(int8_t)))
	    {
	      return (FAIL);
	    }
	}
    }

  config->sync_done = 1;

  vp_os_mutex_unlock(&write_sync_mutex);

  return (SUCCESS);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
