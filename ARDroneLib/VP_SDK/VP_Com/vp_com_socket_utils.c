#ifdef __ARMCC_VERSION
#define _SYS_STAT_H
#endif

#include <VP_Com/vp_com_socket.h>
#include <VP_Com/vp_com_error.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_signal.h>

#include <fcntl.h>
#include <errno.h>

#if defined(__linux__) || defined (__elinux__)
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>

#define CYGPKG_NET 1
#endif

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
        close( s );
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

  /* Warn the socket manager that a client is trying to connect ... */
  res = server_socket->select( server_socket, client_socket, VP_COM_SOCKET_SELECT_ENABLE, write );

  if( SUCCEED(res) )
  {
    if( server_socket->protocol == VP_COM_TCP )
    {
      client_socket->priv = (void*)accept( s, (struct sockaddr*)&raddr, &l );
      /* Warn the socket manager that the client is now connected. */
      res = server_socket->select( server_socket, client_socket, VP_COM_SOCKET_SELECT_CONNECTED, write );
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

  static uint8_t *local_buffer=NULL ;//[VP_COM_THREAD_LOCAL_BUFFER_MAX_SIZE];
  static uint8_t *new_buffer=NULL;
  static int32_t local_buffer_length=0;

  struct sockaddr_in from;
  socklen_t fromlen;
  int32_t s, received,available;

  s = (int32_t) client_socket->priv;

  fromlen = sizeof(from);

  switch(client_socket->protocol)
  {
  case VP_COM_TCP:
	  /* TCP : read as much information as we can */

	  /* Resize the buffer */
	  if (local_buffer==NULL || local_buffer_length<VP_COM_THREAD_LOCAL_BUFFER_SIZE)	  {
		  new_buffer=(uint8_t *)vp_os_realloc(local_buffer,VP_COM_THREAD_LOCAL_BUFFER_SIZE);
		  if (new_buffer==NULL) { PRINT("Not enough memory to store TCP data.\n"); return;  }
		  local_buffer = new_buffer;
		  local_buffer_length = VP_COM_THREAD_LOCAL_BUFFER_SIZE;
	  }


	  /* Fetch data */
	  received = recvfrom(s, (char*)local_buffer,local_buffer_length, 0, (struct sockaddr*)&from, &fromlen);

	  /* We close the socket if an error occurred */
	  if( received <= 0)
	   {
		  DEBUG_PRINT("\nClosing socket on port %i.\n",client_socket->port);
	     client_socket->select( client_socket->server, client_socket, VP_COM_SOCKET_SELECT_DISABLE, (Write) vp_com_write_socket );
	     close( s );
	     vp_os_memset( client_socket, 0, sizeof(vp_com_socket_t) );
	     client_socket->is_disable = TRUE;
	   }
	   /* If some data were received, we pass them to the application.  */
	   else if( received>=0 && client_socket->read != NULL )
	   {
	     client_socket->remotePort = ntohs(from.sin_port);
	     client_socket->read( (void*) client_socket, local_buffer, &received, (&from)->sin_addr.s_addr );
	   }

	  break; // End of TCP processing

  case VP_COM_UDP:
	  /* UDP : read one packet */

	  /* Query packet size */
	  available = recvfrom(s, (char*)local_buffer,0, MSG_PEEK|MSG_TRUNC, (struct sockaddr*)&from, &fromlen);

	  /* Resize the buffer */
		  if(available>0){
			  if (available>VP_COM_THREAD_LOCAL_BUFFER_MAX_SIZE) {
				  PRINT("UDP packet is bigger than maximum authorized size. Dropping packet.\n"); return;
			  }
			  if (local_buffer==NULL || local_buffer_length<available)	  {
				  new_buffer=(uint8_t *)vp_os_realloc(local_buffer,available);
				  if (new_buffer==NULL) {
					  PRINT("UDP packet is bigger than available memory. Dropping packet.\n"); return;
				  }
				  local_buffer = new_buffer; local_buffer_length = available;
		  }}
		  else { return ; }

	  /* Fetch data */
		received = recvfrom(s, (char*)local_buffer,local_buffer_length, 0, (struct sockaddr*)&from, &fromlen);


	  /* Closes the socket if an error occurred on an UDP socket   */
		  if( received < 0 )
		  {
			DEBUG_PRINT("\nClosing socket on port %i.\n",client_socket->port);
			client_socket->select( client_socket->server, client_socket, VP_COM_SOCKET_SELECT_DISABLE, (Write) vp_com_write_socket );
			close( s );
			vp_os_memset( client_socket, 0, sizeof(vp_com_socket_t) );
			client_socket->is_disable = TRUE;
		  }

	  /* If some data were received, we pass them to the application.  */
		  else if( received>=0 && client_socket->read != NULL )
		  {
            client_socket->remotePort = ntohs(from.sin_port);
            client_socket->read( (void*) client_socket, local_buffer, &received, (&from)->sin_addr.s_addr );
		  }

	  break;  // End of UDP processing

  default: break;

  }

  //DEBUG_STEPH  - debugging the fact that an empty UDP packet crashes the Mykonos program

  /* PRINT("Received (%i bytes) from (ip %i.%i.%i.%i) on (port %i) - ",
		  received,
		  ((((struct sockaddr_in*)&from)->sin_addr.s_addr))&0x0FF,
		  ((((struct sockaddr_in*)&from)->sin_addr.s_addr)>>8)&0x0FF,
		  ((((struct sockaddr_in*)&from)->sin_addr.s_addr)>>16)&0x0FF,
		  ((((struct sockaddr_in*)&from)->sin_addr.s_addr)>>24)&0x0FF,
		  ntohs(((struct sockaddr_in*)&from)->sin_port)
		  );

	  switch(client_socket->protocol){
	  case VP_COM_UDP: printf(" UDP"); break;
	  case VP_COM_TCP: printf(" TCP"); break;
	  default : printf("unknown\n"); break;
  	  }
*/

}
