/**
 *  \brief    Com Api for video sdk. Private definitions.
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.com>
 *  \version  1.0
 *  \date     24/07/2007
 */

#ifndef _VP_COM_SERIAL_H_
#define _VP_COM_SERIAL_H_

#if !defined(__NDS__) && !defined(__MACOSX__)

#include <VP_Com/vp_com.h>

#define VP_COM_SYNC_DELAY  100
#define VP_COM_SYNC_STRING "VP_C0M_SERIAL_SYNCHR0"

/// Init & Shutdown Com
C_RESULT vp_com_serial_init(void);
C_RESULT vp_com_serial_shutdown(void);

C_RESULT vp_com_serial_local_config(vp_com_serial_config_t* config);

/// Connect to a server (client side)
C_RESULT vp_com_serial_connect(vp_com_t* vp_com, vp_com_connection_t* connection, int32_t numAttempts);
C_RESULT vp_com_serial_disconnect(vp_com_serial_config_t* config, vp_com_connection_t* connection);

/// Open & close sockets
C_RESULT vp_com_serial_open(vp_com_serial_config_t* config, vp_com_connection_t* connection, vp_com_socket_t* socket, Read* read, Write* write);
C_RESULT vp_com_serial_close(vp_com_socket_t* socket);

/// Listen for incomming connections (server side)
C_RESULT vp_com_serial_wait_connections(vp_com_connection_t** c, vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength);

/// Utility functions
C_RESULT vp_com_serial_network_adapter_lookup(vp_com_network_adapter_lookup_t callback);
C_RESULT vp_com_serial_inquire(const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout);

#endif // ! __NDS__ && ! __MACOSX__

#endif // _VP_COM_SERIAL_H_
