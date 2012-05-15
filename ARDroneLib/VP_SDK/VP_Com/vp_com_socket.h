#ifndef _VP_COM_SOCKET_H_
#define _VP_COM_SOCKET_H_

#include <VP_Com/vp_com.h>

C_RESULT vp_com_open_socket( vp_com_socket_t* sck, Read* read, Write* write );

C_RESULT vp_com_close_socket( vp_com_socket_t* socket );

C_RESULT vp_com_wait_socket( vp_com_socket_t* server, vp_com_socket_t* client, int32_t queueLength );

/// If the destination of your writes for your udp socket change, you have to call this function to notify the socket
C_RESULT vp_com_make_udp_target( vp_com_socket_t* sck );

/// Read / Write functions that vp_com will return when you open an udp socket
C_RESULT vp_com_read_udp_socket(vp_com_socket_t* sck, int8_t* buffer, int32_t* size);
C_RESULT vp_com_write_udp_socket(vp_com_socket_t* sck, const int8_t* buffer, int32_t* size);

/// Read / Write functions that vp_com will return when you open a tcp socket
C_RESULT vp_com_read_socket( vp_com_socket_t* socket, int8_t* buffer, int32_t* size );
C_RESULT vp_com_write_socket( vp_com_socket_t* socket, const int8_t* buffer, int32_t* size );

C_RESULT vp_com_sockopt_ip(vp_com_t* vp_com, vp_com_socket_t* socket, VP_COM_SOCKET_OPTIONS options);

#endif // _VP_COM_IP_H_
