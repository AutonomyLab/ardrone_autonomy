/**
 *  \brief    Com Api for video sdk. Private definitions.
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     22/01/2008
 */

#ifndef _VP_COM_WIRED_H_
#define _VP_COM_WIRED_H_

#include <VP_Com/vp_com.h>

/// Init & Shutdown Com
C_RESULT vp_com_wired_init(void);
C_RESULT vp_com_wired_shutdown(void);

/// Config local interface
C_RESULT vp_com_wired_local_config(vp_com_wired_config_t* config);

/// Connect to a server (client side)
C_RESULT vp_com_wired_connect(vp_com_t* vp_com, vp_com_wired_connection_t* connection);
C_RESULT vp_com_wired_disconnect(vp_com_wired_config_t* config, vp_com_wired_connection_t* connection);

/// Open & close sockets
C_RESULT vp_com_wired_open(vp_com_wired_config_t* config, vp_com_wired_connection_t* connection, vp_com_socket_t* socket, Read* read, Write* write);
C_RESULT vp_com_wired_close(vp_com_socket_t* socket);

/// Listen for incomming connections (server side)
C_RESULT vp_com_wired_wait_connections(vp_com_wired_connection_t** c, vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength);

/// Utility functions
C_RESULT vp_com_wired_network_adapter_lookup(vp_com_network_adapter_lookup_t callback);
C_RESULT vp_com_wired_inquire(const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout);

#endif // _VP_COM_WIRED_H_
