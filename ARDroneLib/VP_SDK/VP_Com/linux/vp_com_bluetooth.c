
#include <netdb.h>
#include <sys/socket.h>

#include <net/if.h>
#include <netinet/in.h>

#include <unistd.h>
#include <stdlib.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>

#include <VP_Com/vp_com_error.h>
#include <VP_Com/vp_com_socket.h>

#include "bluez.h"
#include "vp_com_bluetooth.h"
#include "vp_com_config_itf.h"

#define SOCKET_LISTEN_QUEUE_LENGTH 1

extern char vp_com_pin_code[VP_COM_NAME_MAXSIZE];

static C_RESULT vp_com_open_rfcomm_socket( vp_com_bluetooth_connection_t* connection, vp_com_socket_t* sck, Read* read, Write* write );

C_RESULT vp_com_bt_init( void )
{
  return bluez_init();
}

C_RESULT vp_com_bt_shutdown( void )
{
  return VP_COM_OK;
}

static int hciDevCallback( int dd, int dev_id, long arg )
{
  vp_com_network_adapter_lookup_t callback = ( vp_com_network_adapter_lookup_t ) arg;
  struct hci_dev_info hciDevInfo;

  hci_devinfo( dev_id, &hciDevInfo );
  callback( hciDevInfo.name );

  return 0; // just to remove warning
}

C_RESULT vp_com_bt_network_adapter_lookup( vp_com_network_adapter_lookup_t callback )
{
  hci_for_each_dev( 0, hciDevCallback, (long)callback );

  return VP_COM_OK;
}

C_RESULT vp_com_bt_inquire( const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout )
{
  int i = 0;
  int num_rsp = 0;
  int dd = -1;
  inquiry_info* info = NULL;

  int devid = hci_devid( deviceName );

  if(devid < 0)
    return VP_COM_INITERROR;

  dd = hci_open_dev( devid );
  if(dd < 0)
    return VP_COM_ADAPTORERROR;

  timeout = timeout / 1280;

  num_rsp = hci_inquiry( devid, timeout, num_rsp, NULL, &info, 0 );
  for(i = 0;i < num_rsp;i++){
    bdaddr_t addr;
    char name[248] = {0};
    vp_os_memset( name, 0, sizeof( name ) );
    hci_read_remote_name( dd, &(info+i)->bdaddr, sizeof(name), name, 0 );

    baswap( &addr, &(info+i)->bdaddr );
    callback( &addr,(unsigned char*)name );
  }

  close( dd );
  vp_os_free( info );

  return VP_COM_OK;
}

C_RESULT vp_com_bt_local_config( vp_com_bluetooth_config_t* cfg )
{
  return VP_COM_OK;
}

static int8_t vp_com_bt_match_itf(const char* itfname)
{
  int32_t i;
  static const char itfref[5] = "bnep?";

  for(i = 0; (i < 4) && (itfname[i] == itfref[i]); i++);

  return itfname[i];
}

static C_RESULT vp_com_local_bt_address(const char* itfname, bdaddr_t* address)
{
  C_RESULT res;
  int devid;

  static char hci_name[4] = "hci?";

  hci_name[3] = vp_com_bt_match_itf(itfname);

  devid = hci_devid( hci_name );

  res = (devid < 0) ? VP_COM_ADAPTORNOTFOUND : VP_COM_OK;
  VP_COM_CHECK( res );

  hci_devba( devid, address );

  return res;
}

C_RESULT vp_com_bt_connect(vp_com_t* vp_com, vp_com_bluetooth_connection_t* connection, int32_t numAttempts)
{
  C_RESULT res;
  bdaddr_t local_address;
  vp_com_bluetooth_config_t* config = (vp_com_bluetooth_config_t*) vp_com->config;

  res = (connection == NULL) ? VP_COM_PARAMERROR : VP_COM_OK;
  VP_COM_CHECK( res );

  res = vp_com_local_bt_address( config->itfName, &local_address );
  VP_COM_CHECK( res );

  res = bluez_create_connection( &local_address, &connection->address );
  VP_COM_CHECK( res );

  // configure l'addresse ip de l'interface choisie
  res = vp_com_config_itf( config->itfName, config->localHost, config->broadcast, config->netmask );
  VP_COM_CHECK( res );

  return res;
}

C_RESULT vp_com_bt_disconnect( vp_com_bluetooth_config_t* config, vp_com_bluetooth_connection_t* connection )
{
  bluez_kill_all_connections();

  return VP_COM_OK;
}

C_RESULT vp_com_bt_wait_connections( vp_com_bluetooth_connection_t** c, vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength )
{
  static vp_com_bluetooth_connection_t connection = { 0 };

  C_RESULT res = VP_COM_OK;

  if(server->type != VP_COM_RFCOMM)
  {
    if( *c == NULL )
      *c = &connection;

    if( connection.is_up != 1 )
    {
      res = bluez_listen_connection( &connection.address );
      VP_COM_CHECK( res );

      connection.is_up = 1;
    }
  }

  return vp_com_wait_socket( server, client, queueLength );
}

C_RESULT vp_com_bt_open( vp_com_bluetooth_config_t* config, vp_com_bluetooth_connection_t* connection, vp_com_socket_t* socket, Read* read, Write* write )
{
  C_RESULT res;

  res = ( socket == NULL ) ? VP_COM_PARAMERROR : VP_COM_OK;
  VP_COM_CHECK( res );

  switch( socket->protocol )
  {
    case VP_COM_RFCOMM:
      res = vp_com_open_rfcomm_socket( connection, socket, read, write );
      break;

    case VP_COM_TCP:
    case VP_COM_UDP:
      res = vp_com_open_socket( socket, read, write );
      break;

    default:
      res = VP_COM_NOTSUPPORTED;
      break;
  }

  return res;
}

C_RESULT vp_com_bt_close( vp_com_socket_t* socket )
{
  return vp_com_close_socket( socket );
}

static C_RESULT vp_com_open_rfcomm_socket( vp_com_bluetooth_connection_t* connection, vp_com_socket_t* sck, Read* read, Write* write )
{
  C_RESULT res;

  int s = socket( AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM );

  struct sockaddr_rc raddr = { 0 };
  raddr.rc_family  = AF_BLUETOOTH;
  raddr.rc_channel = sck->scn;

  res = (s < 0) ? VP_COM_ERROR : VP_COM_OK;
  VP_COM_CHECK( res );

  switch( sck->type )
  {
    case VP_COM_SERVER:
      vp_com_copy_address( BDADDR_ANY, &raddr.rc_bdaddr );
      bind( s, (struct sockaddr*)&raddr, sizeof(struct sockaddr_rc) );
      break;

    case VP_COM_CLIENT:
      vp_com_copy_address( &connection->address, &raddr.rc_bdaddr );
      if(connect( s, (struct sockaddr*)&raddr, sizeof(struct sockaddr_rc) ) != 0)
      {
        close( s );
        res = VP_COM_ERROR;
      }
      break;

    default:
      res = VP_COM_PARAMERROR;
      break;
  }

  VP_COM_CHECK( res );

  sck->priv = (void*) s;

  if(read) *read = (Read) vp_com_read_socket;
  if(write) *write = (Write) vp_com_write_socket;

  return res;
}
