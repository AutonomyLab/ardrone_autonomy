/**
 *  \brief    Com Api for video sdk. Private definitions.
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  3.0
 *  \date     16/03/2007
 */

#ifndef _VP_COM_WIFI_H_
#define _VP_COM_WIFI_H_

#include <VP_Com/vp_com.h>

/// Init & Shutdown Com
C_RESULT vp_com_wf_init(void);
C_RESULT vp_com_wf_shutdown(void);

/// Config local interface
C_RESULT vp_com_wf_local_config(vp_com_wifi_config_t* config);

/// Connect to a server (client side)
C_RESULT vp_com_wf_connect(vp_com_t* vp_com, vp_com_wifi_connection_t* connection, int32_t numAttempts);
C_RESULT vp_com_wf_disconnect(vp_com_wifi_config_t* config, vp_com_wifi_connection_t* connection);

/// Get stats about current connection
C_RESULT vp_com_wf_get_rssi(vp_com_wifi_config_t* cfg, int32_t* rssi);

/// Open & close sockets
C_RESULT vp_com_wf_open(vp_com_wifi_config_t* config, vp_com_wifi_connection_t* connection, vp_com_socket_t* socket, Read* read, Write* write);
C_RESULT vp_com_wf_close(vp_com_socket_t* socket);

/// Listen for incomming connections (server side)
C_RESULT vp_com_wf_wait_connections(vp_com_wifi_connection_t** c, vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength);

/// Utility functions
C_RESULT vp_com_wf_network_adapter_lookup(vp_com_network_adapter_lookup_t callback);
C_RESULT vp_com_wf_inquire(const char* deviceName, vp_com_inquiry_t callback, uint32_t timeout);

#endif // _VP_COM_WIFI_H_
