#ifdef _WIN32

#include <VP_Com/vp_com_socket.h>
#include <VP_Com/vp_com_error.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_signal.h>

#include <fcntl.h>
#include <errno.h>


typedef int socklen_t;

int32_t vp_com_fill_read_fs(vp_com_socket_t* sockets, int32_t num_sockets, int32_t max, fd_set* read_fs )
{
  while( num_sockets > 0 )
  {
    if( !sockets->is_disable )
    {
      int32_t s = (int32_t) sockets->priv;

      FD_SET( s, read_fs); // add the socket

      if( s > max )
        max = s;
    }

    sockets ++;
    num_sockets--;
  }

  return max;
}

void vp_com_close_client_sockets(vp_com_socket_t* client_sockets, int32_t num_client_sockets)
{
  int32_t s;

  // Select timed out - We close all sockets because it should mean we lost connection with client
  while( num_client_sockets > 0 )
  {
    if( !client_sockets->is_disable )
    {
      s = (int32_t) client_sockets->priv;

      DEBUG_PRINT_SDK("[VP_COM_SERVER] Closing socket %d\n", (int)s);

      client_sockets->select( client_sockets->server,
                              client_sockets,
                              VP_COM_SOCKET_SELECT_DISABLE,
                              (Write) vp_com_write_socket );

      if( client_sockets->protocol == VP_COM_TCP )
      {
        closesocket( s );
      }

      vp_os_memset( client_sockets, 0, sizeof(vp_com_socket_t) );
      client_sockets->is_disable = TRUE;
    }

    client_sockets++;
    num_client_sockets--;
  }
}

C_RESULT vp_com_client_open_socket(vp_com_socket_t* server_socket, vp_com_socket_t* client_socket)
{
  C_RESULT res;
  struct sockaddr_in raddr = { 0 }; // remote address

  socklen_t l = sizeof(raddr);
  int32_t s = (int32_t) server_socket->priv;

  Write write = (Write) (server_socket->protocol == VP_COM_TCP ? vp_com_write_socket : vp_com_write_udp_socket);

  vp_os_memcpy( client_socket, server_socket, sizeof(vp_com_socket_t) );

  res = server_socket->select( server_socket, client_socket, VP_COM_SOCKET_SELECT_ENABLE, write );

  if( VP_SUCCEEDED(res) )
  {
    if( server_socket->protocol == VP_COM_TCP )
    {
      client_socket->priv = (void*)accept( s, (struct sockaddr*)&raddr, &l );
    }

    DEBUG_PRINT_SDK("[VP_COM_SERVER] Opening socket for server %d\n", (int)s);

    client_socket->server  = server_socket;
  }
  else
  {
    DEBUG_PRINT_SDK("[VP_COM_SERVER] Failed to open socket for server %d\n", (int)s);
    vp_os_memset( client_socket, 0, sizeof(vp_com_socket_t) );
  }

  return res;
}

void vp_com_client_receive( vp_com_socket_t *client_socket )
{
  static int8_t local_buffer[VP_COM_THREAD_LOCAL_BUFFER_MAX_SIZE];
  struct sockaddr from;
  socklen_t fromlen;
  int32_t s, received;

  s = (int32_t) client_socket->priv;

  fromlen = sizeof(from);
  received = recvfrom(s, (char*)local_buffer, sizeof(local_buffer)/*VP_COM_THREAD_LOCAL_BUFFER_MAX_SIZE*/, 0, &from, &fromlen);

  if( received == 0 )
  {
    client_socket->select( client_socket->server, client_socket, VP_COM_SOCKET_SELECT_DISABLE, (Write) vp_com_write_socket );
    closesocket( s );
    vp_os_memset( client_socket, 0, sizeof(vp_com_socket_t) );
    client_socket->is_disable = TRUE;
  }
  else if( client_socket->read != NULL )
  {
    client_socket->read( (void*) client_socket, local_buffer, &received, ((struct sockaddr_in*)&from)->sin_addr.s_addr );
  }
}


#endif
