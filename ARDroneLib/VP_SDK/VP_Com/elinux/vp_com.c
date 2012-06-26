#include <time.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_signal.h>
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Com/vp_com.h>
#include <VP_Com/vp_com_socket.h>
#include <VP_Com/vp_com_error.h>
#include <VP_Api/vp_api_error.h>

#include "vp_com_wifi.h"
#include "vp_com_bluetooth.h"
#include "vp_com_serial.h"
#include "vp_com_wired.h"

typedef C_RESULT (*VP_COM_x_init)(void);
typedef C_RESULT (*VP_COM_x_shutdown)(void);
typedef C_RESULT (*VP_COM_x_network_adapter_lookup)(vp_com_network_adapter_lookup_t callback);
typedef C_RESULT (*VP_COM_x_inquire)(const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout);
typedef C_RESULT (*VP_COM_x_local_config)(vp_com_config_t* config);
typedef C_RESULT (*VP_COM_x_connect)(vp_com_t* vp_com, vp_com_connection_t* connection, int32_t numAttempts);
typedef C_RESULT (*VP_COM_x_disconnect)(vp_com_config_t* config, vp_com_connection_t* connection);
typedef C_RESULT (*VP_COM_x_get_rssi)(vp_com_config_t* cfg, int32_t* rssi);
typedef C_RESULT (*VP_COM_x_wait_connections)(vp_com_connection_t** c, vp_com_socket_t* server, vp_com_socket_t* client, int queueLength);
typedef C_RESULT (*VP_COM_x_open)(vp_com_config_t* config, vp_com_connection_t* connection, vp_com_socket_t* socket, Read* read, Write* write);
typedef C_RESULT (*VP_COM_x_close)(vp_com_socket_t* socket);

#define VP_COM_INIT                  vp_com->init
#define VP_COM_SHUTDOWN              vp_com->shutdown
#define VP_COM_NETWORKADAPTERLOOKUP  vp_com->network_adapter_lookup
#define VP_COM_INQUIRE               vp_com->inquire
#define VP_COM_LOCAL_CONFIG          vp_com->local_config
#define VP_COM_CONNECT               vp_com->connect
#define VP_COM_DISCONNECT            vp_com->disconnect
#define VP_COM_GET_RSSI              vp_com->get_rssi
#define VP_COM_WAITCONNECTIONS       vp_com->wait_connections
#define VP_COM_OPEN                  vp_com->open
#define VP_COM_CLOSE                 vp_com->close

char vp_com_pin_code[VP_COM_NAME_MAXSIZE];

C_RESULT vp_com_init(vp_com_t* vp_com)
{
  C_RESULT res = VP_COM_NOTSUPPORTED;

  VP_OS_ASSERT( vp_com != NULL );

  if(!vp_com->initialized)
  {
    vp_os_mutex_init(&vp_com->mutex);
    vp_com->initialized ++;
  }

  vp_os_mutex_lock(&vp_com->mutex);

  if(vp_com->ref_count > 0)
  {
    vp_com->ref_count ++;
    res = VP_COM_OK;
  }
  else
  {
#ifdef USE_WIFI
     if(vp_com->type == VP_COM_WIFI) 
     {
       vp_com->init                    = (VP_COM_x_init) vp_com_wf_init; 
       vp_com->shutdown                = (VP_COM_x_shutdown) vp_com_wf_shutdown; 
       vp_com->network_adapter_lookup  = (VP_COM_x_network_adapter_lookup) vp_com_wf_network_adapter_lookup; 
       vp_com->local_config            = (VP_COM_x_local_config) vp_com_wf_local_config; 
       vp_com->inquire                 = (VP_COM_x_inquire) vp_com_wf_inquire; 
       vp_com->connect                 = (VP_COM_x_connect) vp_com_wf_connect; 
       vp_com->disconnect              = (VP_COM_x_disconnect) vp_com_wf_disconnect; 
       vp_com->get_rssi                = (VP_COM_x_get_rssi) vp_com_wf_get_rssi; 
       vp_com->wait_connections        = (VP_COM_x_wait_connections) vp_com_wf_wait_connections; 
       vp_com->open                    = (VP_COM_x_open) vp_com_wf_open; 
       vp_com->close                   = (VP_COM_x_close) vp_com_wf_close; 
     }
#endif // > USE_WIFI

#ifdef USE_SERIAL
    if(vp_com->type == VP_COM_SERIAL)
    {
      vp_com->init                    = (VP_COM_x_init) vp_com_serial_init;
      vp_com->shutdown                = (VP_COM_x_shutdown) vp_com_serial_shutdown;
      vp_com->network_adapter_lookup  = (VP_COM_x_network_adapter_lookup) vp_com_serial_network_adapter_lookup;
      vp_com->local_config            = (VP_COM_x_local_config) vp_com_serial_local_config;
      vp_com->inquire                 = (VP_COM_x_inquire) vp_com_serial_inquire;
      vp_com->connect                 = (VP_COM_x_connect) vp_com_serial_connect;
      vp_com->disconnect              = (VP_COM_x_disconnect) vp_com_serial_disconnect;
      vp_com->get_rssi                = NULL;
      vp_com->wait_connections        = (VP_COM_x_wait_connections) vp_com_serial_wait_connections;
      vp_com->open                    = (VP_COM_x_open) vp_com_serial_open;
      vp_com->close                   = (VP_COM_x_close) vp_com_serial_close;
    }
#endif

#ifdef USE_WIRED
    if( vp_com->type == VP_COM_WIRED )
    {
      vp_com->init                    = (VP_COM_x_init) vp_com_wired_init;
      vp_com->shutdown                = (VP_COM_x_shutdown) vp_com_wired_shutdown;
      vp_com->network_adapter_lookup  = (VP_COM_x_network_adapter_lookup) vp_com_wired_network_adapter_lookup;
      vp_com->local_config            = (VP_COM_x_local_config) vp_com_wired_local_config;
      vp_com->inquire                 = (VP_COM_x_inquire) vp_com_wired_inquire;
      vp_com->connect                 = (VP_COM_x_connect) vp_com_wired_connect;
      vp_com->disconnect              = (VP_COM_x_disconnect) vp_com_wired_disconnect;
      vp_com->get_rssi                = NULL;
      vp_com->wait_connections        = (VP_COM_x_wait_connections) vp_com_wired_wait_connections;
      vp_com->open                    = (VP_COM_x_open) vp_com_wired_open;
      vp_com->close                   = (VP_COM_x_close) vp_com_wired_close;
    }
#endif

    if(VP_COM_INIT)
      res = VP_COM_INIT();

    if(res == VP_COM_OK)
    {
      vp_os_install_error_handler( VP_COM_SDK_SIGNATURE, vp_com_formatMessage );
      vp_com->ref_count ++;
    }
  }

  vp_os_mutex_unlock( &vp_com->mutex );

  return res;
}

C_RESULT vp_com_shutdown(vp_com_t* vp_com)
{
  VP_OS_ASSERT( vp_com != NULL );

  vp_os_mutex_lock( &vp_com->mutex );

  if(vp_com->ref_count > 0)
  {
    vp_com->ref_count--;
    if(vp_com->ref_count == 0)
    {
      vp_os_mutex_unlock( &vp_com->mutex );
      vp_os_mutex_destroy( &vp_com->mutex );

      return VP_COM_SHUTDOWN();
    }
  }

  vp_os_mutex_unlock(&vp_com->mutex);

  return VP_COM_OK;
}

// Convert a char to an hexidecimal value between 0x0 and 0xF
static char ctohex(char c)
{
  if(c >= '0' && c <= '9')
    return c - '0';
  if(c >= 'a' && c <= 'f')
    return c - 'a' + 0xa;
  if(c >= 'A' && c <= 'F')
    return c - 'A' + 0xa;

  return 0xFF;
}

// Convert an hexidecimal value to char
static char hextoc(char h)
{
  if(h < 0xa)
    return h + '0';

  return ((h - 0xa) + 'a');
}

C_RESULT vp_com_str_to_address(const char* address, bdaddr_t* addr)
{
  int i = 0;
  while(*address)
  {
    if(*address == ':')
      address++;

    addr->b[i]  = ctohex(*address++) << 4;
    addr->b[i] |= ctohex(*address++);

    i ++;
  }

  return VP_COM_OK;
}

C_RESULT vp_com_address_to_str(const bdaddr_t* addr, char* address)
{
  int i = 0;
  char c1 = addr->b[i] & 0xF0;
  char c2 = addr->b[i] & 0x0F;

  *address++ = hextoc(c1 >> 4);
  *address++ = hextoc(c2);

  for(i = 1;i < 6;i++)
  {
    c1 = addr->b[i] & 0xF0;
    c2 = addr->b[i] & 0x0F;

    *address++ = ':';
    *address++ = hextoc(c1 >> 4);
    *address++ = hextoc(c2);
  }

  return VP_COM_OK;
}

C_RESULT vp_com_copy_address(const bdaddr_t* from,bdaddr_t* to)
{
  vp_os_memcpy( to, from, sizeof( bdaddr_t ) );

  return VP_COM_OK;
}

C_RESULT vp_com_cmp_address(const bdaddr_t* bd1, const bdaddr_t* bd2)
{
  int32_t i;

  for( i = 0; i < BDADDR_SIZE && ( bd1->b[i] == bd2->b[i] ); i++ );

  return ( i < BDADDR_SIZE ) ? VP_COM_ERROR : VP_COM_OK;
}

C_RESULT vp_com_network_adapter_lookup(vp_com_t* vp_com, vp_com_network_adapter_lookup_t callback)
{
  return VP_COM_NETWORKADAPTERLOOKUP( callback );
}

C_RESULT vp_com_local_config(vp_com_t* vp_com, vp_com_config_t* config)
{
  C_RESULT res = C_OK;

  VP_OS_ASSERT( vp_com != NULL );

  if( vp_com->config != config )
  {

    res = VP_COM_LOCAL_CONFIG(config);

    if( SUCCEED( res ) )
      vp_com->config = config;

  }

  return res;
}

C_RESULT vp_com_inquire(vp_com_t* vp_com, const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout)
{
  VP_OS_ASSERT( vp_com != NULL );

  return VP_COM_INQUIRE( deviceName, callback, timeout );
}

C_RESULT vp_com_connect(vp_com_t* vp_com, vp_com_connection_t* connection, uint32_t numAttempts)
{
  C_RESULT res = VP_COM_OK;
  bool_t already_connected;

  VP_OS_ASSERT( vp_com != NULL );

  if(vp_com->config != NULL)
  {
    vp_os_mutex_lock(&vp_com->mutex);

    already_connected = vp_com->connection && vp_com->connection->is_up == 1;

    // TODO voir pour ajouter un test sur l'adresse ethernet de la connection
    if( already_connected && vp_com->connection != connection )
    {
      already_connected = FALSE;
      vp_com_disconnect(vp_com);
    }

    if( !already_connected )
    {
      res = VP_COM_CONNECT(vp_com, connection, numAttempts);

      if( SUCCEED( res ) )
      {
        vp_com->connection = connection;
        vp_com->connection->is_up = 1;
      }
    }

    vp_os_mutex_unlock(&vp_com->mutex);
  }

  return res;
}

C_RESULT vp_com_disconnect(vp_com_t* vp_com)
{
  C_RESULT res = VP_COM_ERROR;

  VP_OS_ASSERT( vp_com != NULL );

  if(vp_com->config != NULL && vp_com->connection != NULL)
  {
    vp_os_mutex_lock(&vp_com->mutex);

    res = VP_COM_DISCONNECT(vp_com->config, vp_com->connection);

    if( SUCCEED( res ) )
      vp_com->connection->is_up = 0;


    vp_os_mutex_unlock(&vp_com->mutex);
  }

  return res;
}

C_RESULT vp_com_get_rssi(vp_com_t* vp_com, int32_t* rssi)
{
  C_RESULT res;

  if( vp_com != NULL && vp_com->config != NULL && vp_com->get_rssi != NULL )
    res = VP_COM_GET_RSSI( vp_com->config, rssi );
  else
  {
    *rssi = 0;
    res = C_FAIL;
  }

  return res;
}

C_RESULT vp_com_wait_connections(vp_com_t* vp_com, vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength)
{
  VP_OS_ASSERT( vp_com != NULL );

  return VP_COM_WAITCONNECTIONS( &vp_com->connection, server, client, queueLength );
}

C_RESULT vp_com_open(vp_com_t* vp_com, vp_com_socket_t* socket,Read* read,Write* write)
{
  VP_OS_ASSERT( vp_com != NULL );

  if( vp_com->ref_count > 0 )
    return VP_COM_OPEN(vp_com->config, vp_com->connection, socket, read, write);

  PRINT("[VP_COM] Trying to open a socket with a non initialized vp_com object\n");

  return C_FAIL;
}

C_RESULT vp_com_close(vp_com_t* vp_com, vp_com_socket_t* socket)
{
  VP_OS_ASSERT( vp_com != NULL );

  if( vp_com->ref_count > 0 )
    return VP_COM_CLOSE( socket );

  PRINT("[VP_COM] Trying to close a socket with a non initialized vp_com object\n");

  return C_FAIL;
}

C_RESULT vp_com_sockopt(vp_com_t* vp_com, vp_com_socket_t* socket, VP_COM_SOCKET_OPTIONS options)
{
  C_RESULT res;

  switch( socket->protocol )
  {
#ifndef NO_COM
    case VP_COM_TCP:
      res = vp_com_sockopt_ip(vp_com, socket, options);
      break;
#endif
    case VP_COM_SERIAL:
      res = VP_COM_OK;
      break;

    default:
      res = VP_COM_ERROR;
      break;
  }

  return res;
}
