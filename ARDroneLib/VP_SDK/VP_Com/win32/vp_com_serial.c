// Header ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 *  \brief    Com Api for video sdk. Private declarations.
 *  \author   Mayeul Rigo <mayeul.rigo@parrot.com>
 *  \version  1.0
 *  \date     11/02/09
 */

//pour les ports de COM >= 10 utiliser la syntaxe \\\\.\\COM10 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Include //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <errno.h>

#include <VP_Com/vp_com.h>
#include <VP_Com/vp_com_error.h>

#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_delay.h>

#include "vp_com_serial.h"

// Static ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static DCB tio_save;
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
  DCB tio;
  COMMTIMEOUTS commtimeouts;

  VP_OS_ASSERT(config->blocking == 0 || config->blocking == 1);

  socket->priv = (void *)CreateFile(&config->itfName[0],GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
  if (socket->priv == INVALID_HANDLE_VALUE) return VP_COM_ERROR;

  if(config->blocking == 0) {
    commtimeouts.ReadIntervalTimeout         = 0;
    commtimeouts.ReadTotalTimeoutMultiplier  = 0;
    commtimeouts.ReadTotalTimeoutConstant    = 1;
    commtimeouts.WriteTotalTimeoutMultiplier = 0;
    commtimeouts.WriteTotalTimeoutConstant   = 1;
  } else {
    commtimeouts.ReadTotalTimeoutMultiplier  = 0;
    commtimeouts.ReadTotalTimeoutConstant    = 0;
    commtimeouts.WriteTotalTimeoutMultiplier = 0;
    commtimeouts.WriteTotalTimeoutConstant   = 0;
  };
  if (SetCommTimeouts(socket->priv,&commtimeouts) == 0) return VP_COM_ERROR;

  if(((int)socket->priv) == 0)
    {
      PRINT("Unable to \"open\" serial device");
      return VP_COM_ERROR;
    }

  /* get current serial port settings */
  if (GetCommState(socket->priv,&tio) == 0) {
    PRINT("Serial device configuration failure (119) (%s)", strerror(errno));
    CloseHandle((HANDLE)socket->priv);
    return (VP_COM_ERROR);
  };
  tio_save = tio;

  if(config->sync) {
    /* set serial settings */
    tio.BaudRate          = config->initial_baudrate;
    tio.fBinary           = TRUE;
    tio.fParity           = FALSE;
    tio.fOutxCtsFlow      = FALSE;
    tio.fOutxDsrFlow      = FALSE;
    tio.fDtrControl       = DTR_CONTROL_DISABLE;
    tio.fDsrSensitivity   = FALSE;
    tio.fTXContinueOnXoff = TRUE;
    tio.fOutX             = FALSE;
    tio.fInX              = FALSE;
    tio.fErrorChar        = FALSE;
    tio.fNull             = FALSE;
    tio.fRtsControl       = RTS_CONTROL_DISABLE;
    tio.fAbortOnError     = FALSE;
    tio.Parity            = NOPARITY;
    tio.StopBits          = ONESTOPBIT;

    if (SetCommState(socket->priv,&tio) == 0) {
  	  PRINT("Serial device configuration failure (145) (%s)", strerror(errno));
      CloseHandle((HANDLE)socket->priv);
	    return (VP_COM_ERROR);
	  };

    if(socket->type == VP_COM_CLIENT) {
  	  if(FAILED(vp_com_serial_write_sync(config, socket))) return (VP_COM_ERROR);
	    vp_os_delay(VP_COM_SYNC_DELAY);
	  } else if(socket->type == VP_COM_SERVER) {
      if(FAILED(vp_com_serial_wait_sync(config, socket))) return (VP_COM_ERROR);
      vp_os_delay(VP_COM_SYNC_DELAY);
    }
  }

  tio.BaudRate          = config->baudrate;
  tio.fBinary           = TRUE;
  tio.fParity           = FALSE;
  tio.fOutxCtsFlow      = FALSE;
  tio.fOutxDsrFlow      = FALSE;
  tio.fDtrControl       = DTR_CONTROL_DISABLE;
  tio.fDsrSensitivity   = FALSE;
  tio.fTXContinueOnXoff = TRUE;
  tio.fOutX             = FALSE;
  tio.fInX              = FALSE;
  tio.fErrorChar        = FALSE;
  tio.fNull             = FALSE;
  tio.fRtsControl       = RTS_CONTROL_DISABLE;
  tio.fAbortOnError     = FALSE;
  tio.Parity            = NOPARITY;
  tio.StopBits          = ONESTOPBIT;

  if (SetCommState(socket->priv,&tio) == 0) {
    PRINT("Serial device configuration failure (177) (%s)", strerror(errno));
    CloseHandle((HANDLE)socket->priv);
    return (VP_COM_ERROR);
  };

  if(read) *read = (Read) vp_com_serial_read;
  if(write) *write = (Write) vp_com_serial_write;

  return (VP_COM_OK);
}

C_RESULT vp_com_serial_close(vp_com_socket_t* socket)
{
  if(SetCommState(socket->priv,&tio_save) == 0)
    {
      PRINT("Serial device configuration failure (191) (%s)", strerror(errno));
      CloseHandle((HANDLE)socket->priv);
      return (VP_COM_ERROR);
    }

    CloseHandle((HANDLE)socket->priv);

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
  uint32_t byte_read = 0;
  if (ReadFile((HANDLE)socket->priv, buffer, *size, &byte_read, 0) == 0) {
    if (errno != EAGAIN) return VP_COM_ERROR;
    else *size = 0;
  } else *size = byte_read;
  return VP_COM_OK;
}

static C_RESULT vp_com_serial_write(vp_com_socket_t* socket, const int8_t* buffer, int32_t* size)
{
  uint32_t remain = *size, written = 0;
  while (remain > 0) {
    if (WriteFile((HANDLE)socket->priv, buffer + (*size-remain), remain, &written, 0) == 0) {
      if (errno != EAGAIN) return VP_COM_ERROR;
      else 	written = 0;
    };
    remain -= written;
  };

  return VP_COM_OK;
}

static C_RESULT vp_com_serial_wait_sync(vp_com_serial_config_t* config, vp_com_socket_t* socket)
{
  uint32_t nb;
  uint8_t c;

  vp_os_mutex_lock(&wait_sync_mutex);

  if (!config->sync_done) {
    nb = 0;
    do {
      if (ReadFile((HANDLE)socket->priv, &c, sizeof(int8_t), 0, 0) == 0) return (FAIL);
	    if (c == (VP_COM_SYNC_STRING)[nb]) {
	      nb++;
	    } else {
	      nb = 0;
	    }
    } while(nb != sizeof(VP_COM_SYNC_STRING));
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

  if(!config->sync_done) {
    for(i = 0 ; i < sizeof(VP_COM_SYNC_STRING) ; i++) {
      c = (VP_COM_SYNC_STRING)[i];
      if (WriteFile((HANDLE)socket->priv, &c, sizeof(int8_t), 0, 0) == 0) return (FAIL);
    }
  }

  config->sync_done = 1;

  vp_os_mutex_unlock(&write_sync_mutex);

  return (SUCCESS);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
