#include <VP_Com/vp_com_socket.h>
#include <VP_Com/vp_com_error.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_signal.h>

#include <fcntl.h>
#include <errno.h>

#ifdef __linux__
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>

#define CYGPKG_NET 1
#endif

#if defined(USE_MINGW32) || defined(TARGET_OS_IPHONE) || defined(TARGET_IPHONE_SIMULATOR)

#define MSG_NOSIGNAL 0

#define CYGPKG_NET 1
#endif // USE_MINGW32 || TARGET_OS_IPHONE || TARGET_IPHONE_SIMULATOR

#ifdef CYGPKG_NET

#ifndef USE_MINGW32
extern int inet_addr(const char *);
#endif

C_RESULT vp_com_open_socket(vp_com_socket_t* sck, Read* read, Write* write)
{
  C_RESULT res = VP_COM_OK;

  int s = -1, type;
  struct sockaddr_in name = { 0 };

  switch( sck->protocol )
  {
    case VP_COM_TCP:
      s = socket( AF_INET, SOCK_STREAM, 0 );
      res = ( s < 0 ) ? VP_COM_ERROR : VP_COM_OK;
      type = sck->type;
      break;

    case VP_COM_UDP:
      s = socket( AF_INET, SOCK_DGRAM, 0 );
      sck->scn  = inet_addr(sck->serverHost); // Cache destination in int format
      res = ( s < 0 ) ? VP_COM_ERROR : VP_COM_OK;
      type = VP_COM_SERVER; // Make sure we will bind the socket in the connection-less protocol
      break;

    default:
      type = VP_COM_CLIENT;
      res = VP_COM_PARAMERROR;
      break;
  }

  if( FAILED(res) )
  {
    PRINT("\nSocket opening failed\n");
  }

  VP_COM_CHECK( res );

  name.sin_family = AF_INET;
  name.sin_port   = htons( sck->port );
  switch( type )
  {
    case VP_COM_CLIENT:
      name.sin_addr.s_addr  = inet_addr(sck->serverHost);
      if ( connect( s, (struct sockaddr*)&name, sizeof( name ) ) == -1 )
        res = VP_COM_ERROR;
      break;

    case VP_COM_SERVER:
      name.sin_addr.s_addr  = INADDR_ANY;

      if ( bind( s, (struct sockaddr*)&name, sizeof(struct sockaddr)) < 0 )
        res = VP_COM_ERROR;

      if ( sck->is_multicast == 1 )
      {

        if ( sck->multicast_base_addr == 0 )
        {
          PRINT("Error : multicast base address is not defined\n");
          res = VP_COM_ERROR;
          break;
        }

        in_addr_t multicast_address, drone_address;
        vp_com_wifi_config_t *wifi_cfg = (vp_com_wifi_config_t*) wifi_config();

        // compute remote address according to local address
    	drone_address = inet_addr(wifi_cfg->server);
        multicast_address = htonl( sck->multicast_base_addr | (ntohl(drone_address) & 0xFF) );

        struct ip_mreq mreq;
        mreq.imr_interface.s_addr = inet_addr(wifi_cfg->localHost);
        mreq.imr_multiaddr.s_addr = multicast_address;

        if ( setsockopt( s, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq) ) < 0) {
          PRINT("!!! Error enabling multicast !!!\n");
        }
      }

      break;

    default:
      res = VP_COM_PARAMERROR;
      break;
  }

  if(res == VP_COM_OK)
  {
    sck->priv = (void*) s;

    switch( sck->protocol )
    {
      case VP_COM_TCP:
        if(read)  *read   = (Read) vp_com_read_socket;
        if(write) *write  = (Write) vp_com_write_socket;
        break;

      case VP_COM_UDP:
        if(read)  *read   = (Read) vp_com_read_udp_socket;
        if(write) *write  = (Write) vp_com_write_udp_socket;
        break;

      default:
        if(read)  *read   = NULL;
        if(write) *write  = NULL;
        break;
    }
  }
  else
  {
    close( s );
  }

  if (sck->block != VP_COM_DEFAULT &&
      sck->block != VP_COM_WAITALL &&
      sck->block != VP_COM_DONTWAIT)
  {
    sck->block = VP_COM_DEFAULT;
  }

  return res;
}

C_RESULT vp_com_close_socket(vp_com_socket_t* socket)
{
  if(socket == NULL)
    return VP_COM_PARAMERROR;

  // shutdown( (int) socket->priv, SHUT_RDWR );
  close( (int) socket->priv );

  socket->priv = NULL;

  return VP_COM_OK;
}

C_RESULT vp_com_wait_socket(vp_com_socket_t* server, vp_com_socket_t* client, int32_t queue_length)
{
  C_RESULT res = VP_COM_OK;

  if(server == NULL)
    return VP_COM_PARAMERROR;

  int s = (int) server->priv;
  int c = 0;
  socklen_t l = sizeof(struct sockaddr_in);
  struct sockaddr_in raddr = { 0 }; // remote address

  server->queue_length = queue_length;

  listen(s, queue_length);
  c = accept( s, (struct sockaddr*)&raddr, &l );
  if( c < 0 )
    res = VP_COM_ERROR;

  if(SUCCEED( res ))
  {
    vp_os_memcpy( client, server, sizeof(vp_com_socket_t) );
    client->priv = (void*) c;
  }

  return res;
}

#endif

C_RESULT vp_com_sockopt_ip(vp_com_t* vp_com, vp_com_socket_t* socket, VP_COM_SOCKET_OPTIONS options)
{
  int32_t one  = 1;
  int32_t zero = 0;

  C_RESULT res = VP_COM_ERROR;
#ifdef CYGPKG_NET
  int s = (int) socket->priv;

  if( options & VP_COM_NON_BLOCKING )
  {
#ifndef USE_MINGW32
    int32_t arg = 1;

    PRINT("Setting socket %d to non blocking\n", s);
    res = ioctl( s, FIONBIO, &arg ) < 0 ? C_FAIL : C_OK;
#endif

    if( FAILED(res) )
      PRINT("error setting non blocking\n");
  }

  if( options & VP_COM_NO_DELAY )
  {
    PRINT("Disabling the Nagle (TCP No Delay) algorithm for socket %d\n", s);

    res = setsockopt( s, IPPROTO_TCP, TCP_NODELAY, (char *)&one, sizeof(one) ) < 0 ? C_FAIL : C_OK;

    if( FAILED(res) )
      PRINT("error disabling the Nagle algorithm\n");
  }

  if ( options & VP_COM_MULTICAST_ON )
  {
    res = setsockopt( s, IPPROTO_IP, IP_MULTICAST_LOOP, &zero, sizeof(zero) );
    if ( FAILED(res) )
      PRINT("Error : cannot set IP_MULTICAST_LOOP to 0\n");

    struct in_addr interface_addr;
    interface_addr.s_addr = INADDR_ANY;
    res = setsockopt( s, IPPROTO_IP, IP_MULTICAST_IF, &interface_addr, sizeof(interface_addr) );
    if ( FAILED(res) ) {
      PRINT("Error : cannot enable multicast.\n");
    }
  }
#endif
  return res;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CYGPKG_NET

C_RESULT vp_com_read_udp_socket(vp_com_socket_t* sck, int8_t* buffer, int32_t* size)
{
  C_RESULT res;
  int s = (int) sck->priv;
  struct sockaddr_in from;

  socklen_t from_len = sizeof(from);

  int flags = MSG_NOSIGNAL;
  if (VP_COM_DONTWAIT == sck->block)
    flags |= MSG_DONTWAIT;
  else if (VP_COM_WAITALL == sck->block)
    flags |= MSG_WAITALL;
    

  if(s >= 0)
  {
    res = VP_COM_OK;
    *size = recvfrom(s, (char*)buffer, *size, flags, (struct sockaddr*)&from, &from_len );

    if(*size < 0)
    {

#ifdef USE_MINGW32
      switch( WSAGetLastError() )
      {
        case WSAEOPNOTSUPP:
          PRINT("MSG_NOSIGNAL is not supported on this platform\n");
          res = VP_COM_ERROR;
          break;

        case WSAEINTR:
          *size = 0;
          break;

        case WSAENETDOWN:
        case WSAETIMEDOUT:
        case WSAECONNRESET:
          PRINT("Connection with peer is not enabled\n");
          res = VP_COM_ERROR;
          break;
      }
#else
      switch( errno )
      {
        case EAGAIN:
        case EINTR:
          *size = 0;
          break;

        case EOPNOTSUPP:
          PRINT("MSG_NOSIGNAL is not supported on this platform\n");
          res = VP_COM_ERROR;
          break;

        case EPIPE:
        case ENOTCONN:
        case ECONNRESET:
          PRINT("Connection with peer is not enabled\n");
          res = VP_COM_ERROR;
          break;

        default:
          PRINT("recvfrom fails with error: %s\n", strerror(errno));
          res = VP_COM_ERROR;
          break;
      }
#endif // USE_MINGW32
    }
    else
    {
      sck->scn   = from.sin_addr.s_addr;
      sck->port  = ntohs(from.sin_port);
    }
  }
  else
  {
    res = VP_COM_ERROR;
  }

  return res;
}

C_RESULT vp_com_make_udp_target( vp_com_socket_t* sck )
{
  C_RESULT res = C_FAIL;

  if( sck->protocol == VP_COM_UDP )
  {
    sck->scn  = inet_addr(sck->serverHost); // We use scn field to store ip in order to avoid a call to inet_addr each time we call write
    res = C_OK;
  }

  return res;
}

C_RESULT vp_com_write_udp_socket(vp_com_socket_t* sck, const int8_t* buffer, int32_t* size)
{
  C_RESULT res;
  int s = (int) sck->priv;
  struct sockaddr_in to;

  int flags = 0;
  if (VP_COM_DONTWAIT == sck->block)
    flags |= MSG_DONTWAIT;
  else if (VP_COM_WAITALL == sck->block)
    flags |= MSG_WAITALL;

  if(s >= 0)
  {
    res = VP_COM_OK;

    vp_os_memset( (char*)&to, 0, sizeof(to) );
    to.sin_family       = AF_INET;
    to.sin_addr.s_addr  = sck->scn;
    if ( sck -> remotePort )
        to.sin_port     = htons(sck->remotePort);
    else
        to.sin_port     = htons(sck->port);

    *size = sendto( s, (char*)buffer, *size, flags, (struct sockaddr*)&to, sizeof(to) );

    if(*size < 0)
    {
#ifdef USE_MINGW32

      switch( WSAGetLastError() )
      {
        case WSAEOPNOTSUPP:
          PRINT("MSG_NOSIGNAL is not supported on this platform\n");
          res = VP_COM_ERROR;
          break;

        case WSAEINTR:
          *size = 0;
          break;

        case WSAENETDOWN:
        case WSAETIMEDOUT:
        case WSAECONNRESET:
          PRINT("Connection with peer is not enabled\n");
          res = VP_COM_ERROR;
          break;
      }

#else
      if( errno == EAGAIN )
      {
        *size = 0;
      }
      else
      {
        switch( errno )
        {
          case EOPNOTSUPP:
            PRINT("MSG_NOSIGNAL is not supported on this platform\n");
            break;

          case EPIPE:
          case ENOTCONN:
          case ECONNRESET:
            PRINT("Connection with peer is not enabled\n");
            break;

          default:
            PRINT("sendto fails with error: %d - %s\n", errno, strerror(errno));
            break;
        }

        res = VP_COM_ERROR;
      }
#endif // USE_MINGW32
    }
  }
  else
  {
    res = VP_COM_ERROR;
  }

  return res;
}

C_RESULT vp_com_read_socket(vp_com_socket_t* socket, int8_t* buffer, int32_t* size)
{
  C_RESULT res;
  int s = (int) socket->priv;

  int flags = 0;
  if (VP_COM_DONTWAIT == socket->block)
    flags |= MSG_DONTWAIT;
  else if (VP_COM_WAITALL == socket->block)
    flags |= MSG_WAITALL;

  if(s >= 0)
  {
    res = VP_COM_OK;
    *size = recv(s, buffer, *size, flags);
    if(*size < 0)
    {
      if( errno == EAGAIN )
      {
        *size = 0;
      }
      else
      {
        res = VP_COM_ERROR;
      }
    }
  }
  else
  {
    res = VP_COM_ERROR;
  }

  return res;
}

C_RESULT vp_com_write_socket(vp_com_socket_t* socket, const int8_t* buffer, int32_t* size)
{
  C_RESULT res;
  int s = (int) socket->priv;

  int flags = 0;
  if (VP_COM_DONTWAIT == socket->block)
    flags |= MSG_DONTWAIT;
  else if (VP_COM_WAITALL == socket->block)
    flags |= MSG_WAITALL;

  if(s >= 0)
  {
    res = VP_COM_OK;
    *size = send(s, buffer, *size, flags);
    if(*size < 0)
    {
      if( errno == EAGAIN )
      {
        *size = 0;
      }
      else
      {
        res = VP_COM_ERROR;
      }
    }
  }
  else
  {
    res = VP_COM_ERROR;
  }

  return res;
}

#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

static vp_os_mutex_t  server_initialisation_mutex;
static vp_os_cond_t   server_initialisation_wait;
static bool_t         server_init_not_finished = FALSE;

C_RESULT vp_com_init_server(void)
{
  server_init_not_finished = TRUE;
  vp_os_mutex_init(&server_initialisation_mutex);
  vp_os_cond_init(&server_initialisation_wait, &server_initialisation_mutex);

  return C_OK;
}

C_RESULT vp_com_wait_for_server_up(void)
{
  if( server_init_not_finished )
  {
    vp_os_mutex_lock(&server_initialisation_mutex);
    vp_os_cond_wait(&server_initialisation_wait);
    vp_os_mutex_unlock(&server_initialisation_mutex);
  }

  return C_OK;
}

C_RESULT vp_com_timed_wait_for_server_up(uint32_t ms)
{
  C_RESULT res = C_OK;

  if( server_init_not_finished )
  {
    vp_os_mutex_lock(&server_initialisation_mutex);
    res = vp_os_cond_timed_wait(&server_initialisation_wait, ms);
    vp_os_mutex_unlock(&server_initialisation_mutex);
  }

  return res;
}

extern int32_t vp_com_fill_read_fs(vp_com_socket_t* sockets, int32_t num_sockets, int32_t max, fd_set* read_fs );
extern void vp_com_close_client_sockets(vp_com_socket_t* client_sockets, int32_t num_client_sockets);
extern C_RESULT vp_com_client_open_socket(vp_com_socket_t* server_socket, vp_com_socket_t* client_socket);
extern void vp_com_client_receive( vp_com_socket_t *client_socket );

DEFINE_THREAD_ROUTINE_STACK( vp_com_server, thread_params, VP_COM_THREAD_SERVER_STACK_SIZE )
{
#ifdef CYGPKG_NET
  vp_com_socket_t client_sockets[VP_COM_THREAD_NUM_MAX_CLIENTS];
  struct timeval tv, *ptv;

  // This thread setup connection then loop & wait for a socket event
  vp_com_server_thread_param_t* params = (vp_com_server_thread_param_t*) thread_params;

  int32_t i, rc, ncs, s, max = 0, num_server_sockets = params->num_servers, num_client_sockets = 0;
  vp_com_socket_t* server_sockets = params->servers;
  fd_set read_fs;

  vp_os_memset( client_sockets, 0, sizeof( client_sockets ));

  if(FAILED(vp_com_init(params->com)))
  {
    DEBUG_PRINT_SDK("[VP_COM_SERVER] Failed to init com\n");
    vp_com_shutdown(params->com);
  }
  else if(FAILED(vp_com_local_config(params->com, params->config)))
  {
    DEBUG_PRINT_SDK("[VP_COM_SERVER] Failed to configure com\n");
    vp_com_shutdown(params->com);
  }
  else if(FAILED(vp_com_connect(params->com, params->connection, 1)))
  {
    DEBUG_PRINT_SDK("[VP_COM_SERVER] Failed to connect\n");
    vp_com_shutdown(params->com);
  }
  else
  {
    vp_os_mutex_lock(&server_initialisation_mutex);
    vp_os_cond_signal(&server_initialisation_wait);
    vp_os_mutex_unlock(&server_initialisation_mutex);

    server_init_not_finished = FALSE;

    for( i = 0; i < num_server_sockets; i++ )
    {
      if(FAILED( vp_com_open_socket(&server_sockets[i], NULL, NULL) ))
      {
        DEBUG_PRINT_SDK("[VP_COM_SERVER] Unable to open server socket\n");
        server_sockets[i].is_disable = TRUE;
      }
      else
      {
        listen((int32_t)server_sockets[i].priv, server_sockets[i].queue_length);
      }
    }

    params->run = TRUE;

    while( params->run == TRUE )
    {
      if( params->timer_enable == FALSE || ( params->wait_sec == 0 && params->wait_usec == 0 ) )
      {
        ptv = NULL;
      }
      else
      {
        tv.tv_sec   = params->wait_sec;
        tv.tv_usec  = params->wait_usec;
        ptv         = &tv;
      }

      FD_ZERO(&read_fs);
      max = vp_com_fill_read_fs( &server_sockets[0], num_server_sockets, 0, &read_fs );
      max = vp_com_fill_read_fs( &client_sockets[0], num_client_sockets, max, &read_fs );

      rc = select( max + 1, &read_fs, NULL, NULL, ptv );
      if( rc == -1 && ( errno == EINTR || errno == EAGAIN ) )
        continue;

      if( rc == 0 )
      {
        DEBUG_PRINT_SDK("[VP_COM_SERVER] select timeout\n");

        vp_com_close_client_sockets(&client_sockets[0], num_client_sockets);
        num_client_sockets = 0;

        params->timer_enable  = FALSE;
        vp_os_memset( client_sockets, 0, sizeof( client_sockets ));
      }

      for( i = 0; i < num_server_sockets && rc != 0; i++ )
      {
        s = (int32_t) server_sockets[i].priv;

        if( ( !server_sockets[i].is_disable ) && FD_ISSET( s, &read_fs) )
        {
          rc --;

          // Recycle previously released sockets
          for( ncs = 0; ncs < num_client_sockets && client_sockets[ncs].priv != NULL; ncs++ );

          if( ncs < VP_COM_THREAD_NUM_MAX_CLIENTS)
          {
            if( SUCCEED(vp_com_client_open_socket(&server_sockets[i], &client_sockets[ncs])) && ( ncs == num_client_sockets ) )
              num_client_sockets ++;
          }
        }
      }

      for( i = 0; i < num_client_sockets && rc != 0; i++ )
      {
        s = (int32_t) client_sockets[i].priv;
        if( ( !client_sockets[i].is_disable ) && FD_ISSET( s, &read_fs) )
        {
          rc--;

          vp_com_client_receive( &client_sockets[i] );
        }
      }
    }

    for( i = 0; i < num_server_sockets; i++ )
    {
      vp_com_close_socket(&server_sockets[i]);
    }
  }

  vp_com_disconnect(params->com);
  vp_com_shutdown(params->com);
#endif

  THREAD_RETURN( 0 );
}
