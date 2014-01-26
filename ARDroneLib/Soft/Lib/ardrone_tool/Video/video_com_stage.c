#include <config.h>

#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Os/vp_os_assert.h>
#include <ardrone_tool/Video/video_com_stage.h>

#include <VP_Com/vp_com_socket.h>

#ifndef _WIN32
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <sys/time.h>
#define SSOPTCAST_RW(x) x
#define SSOPTCAST_RO(x) x
#else
#define SSOPTCAST_RO(x)  (const char*)x
#define SSOPTCAST_RW(x)  (char*)x
typedef int socklen_t;
#endif

#define VIDEO_COM_DEBUG (0)
#if VIDEO_COM_DEBUG
#define PDBG(...)                                               \
  do                                                            \
    {                                                           \
      PRINT ("V_COM_STAGE: %s:%d : ", __FUNCTION__, __LINE__); \
      PRINT (__VA_ARGS__);                                     \
      PRINT ("\n");                                            \
    } while (0)
#else
#define PDBG(...)
#endif

#define SOCKET_BUFFER_SIZE (1024*1024)

/**
 * IPHONE DEBUG ZONE
 */
float DEBUG_bitrate = 0.0;
#define DEBUG_TIME_BITRATE_CALCULATION_MSEC 1000.0
unsigned long DEBUG_totalBytes = 0;
int DEBUG_isTcp = 0;

static vp_os_mutex_t registerMutex;
static video_com_config_t **registeredStages = NULL;
static uint32_t nbRegisteredStages = 0;

const vp_api_stage_funcs_t video_com_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) video_com_stage_open,
  (vp_api_stage_transform_t) video_com_stage_transform,
  (vp_api_stage_close_t) video_com_stage_close
};


const vp_api_stage_funcs_t video_com_multisocket_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) video_com_multisocket_stage_open,
  (vp_api_stage_transform_t) video_com_multisocket_stage_transform,
  (vp_api_stage_close_t) video_com_multisocket_stage_close
};

C_RESULT video_com_stage_register (video_com_config_t *cfg)
{
  static int runOnce = 1;
  if (1 == runOnce)
    {
      vp_os_mutex_init (&registerMutex);
      runOnce = 0;
    }

  vp_os_mutex_lock (&registerMutex);
  nbRegisteredStages++;
  registeredStages = vp_os_realloc (registeredStages, nbRegisteredStages * sizeof (video_com_config_t *));
  C_RESULT retVal = C_FAIL;
  if (NULL != registeredStages)
    {
      registeredStages[nbRegisteredStages - 1] = cfg;
      retVal = C_OK;
    }

  vp_os_mutex_unlock (&registerMutex);
  return retVal;
}

C_RESULT video_com_stage_connect (video_com_config_t *cfg)
{
#ifdef _WIN32
  int timeout_for_windows=1000; /* timeout in milliseconds */
  int sizeinit;
#else
  struct timeval tv;
  // 1 second timeout
  tv.tv_sec   = 1;
  tv.tv_usec  = 0;
#endif

  C_RESULT res = C_FAIL;
	
  if (TRUE == cfg->connected)
    {
      PDBG ("Will close");
      res = vp_com_close (cfg->com, &cfg->socket);
      cfg->connected = FALSE;
      if (VP_FAILED (res))
        {
          PDBG ("Close failed");
          return res;
        }
    }

  if( cfg->protocol == VP_COM_PROBE)
    {
      PRINT("\n\nPROBING\n");

      cfg->socket.protocol = VP_COM_TCP;
      res = vp_com_open(cfg->com, &cfg->socket, &cfg->read, &cfg->write);

      if( VP_SUCCEEDED(res) )
        {
          PRINT("\n\nTCP\n");
          vp_com_close (cfg->com, &cfg->socket);
          cfg->protocol = VP_COM_TCP;
        }
      else
        {
          PRINT("\n\nUDP\n");
          cfg->protocol = VP_COM_UDP;
        }
    }


  if( cfg->protocol == VP_COM_UDP )
    {
      cfg->socket.protocol = VP_COM_UDP;
      cfg->socket.is_multicast = 0; // disable multicast for video
      cfg->socket.multicast_base_addr = MULTICAST_BASE_ADDR;

      res = vp_com_open(cfg->com, &cfg->socket, &cfg->read, &cfg->write);

      if( VP_SUCCEEDED(res) )
    	{
          int numi= 1;
          socklen_t numi1= sizeof(int);
#ifdef _WIN32
          setsockopt((int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVTIMEO, SSOPTCAST_RO(&timeout_for_windows), sizeof(timeout_for_windows));
#else
          setsockopt((int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVTIMEO, SSOPTCAST_RO(&tv), sizeof(tv));
#endif		
          // Increase buffer for receiving datas.
          setsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_DEBUG, SSOPTCAST_RO(&numi), sizeof(numi));
          numi = SOCKET_BUFFER_SIZE;
          setsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVBUF, SSOPTCAST_RO(&numi),numi1);
          getsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVBUF, SSOPTCAST_RW(&numi),&numi1);
          PDBG ("New buffer size : %d", numi);
          numi1 = 0;
          setsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_DEBUG, SSOPTCAST_RO(&numi1), sizeof(numi1));
          cfg->connected = TRUE;
    	}
    }
  else if( cfg->protocol == VP_COM_TCP )
    {
      PDBG ("Will open TCP");
      res = vp_com_open(cfg->com, &cfg->socket, &cfg->read, &cfg->write);

      if( VP_SUCCEEDED(res) )
    	{
          int numi= 1;
          socklen_t numi1= sizeof(int);
          PDBG ("Success open");
          vp_com_sockopt(cfg->com, &cfg->socket, cfg->sockopt);
#ifdef _WIN32
          setsockopt((int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVTIMEO, SSOPTCAST_RO(&timeout_for_windows), sizeof(timeout_for_windows));
#else
          setsockopt((int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVTIMEO, SSOPTCAST_RO(&tv), sizeof(tv));
#endif		
          // Increase buffer for receiving datas.
          setsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_DEBUG, SSOPTCAST_RO(&numi), sizeof(numi));
          numi = SOCKET_BUFFER_SIZE;
          setsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVBUF, SSOPTCAST_RO(&numi),numi1);
          getsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVBUF, SSOPTCAST_RW(&numi),&numi1);
          PDBG ("NEW buffer size : %d", numi);
          numi1 = 0;
          setsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_DEBUG, SSOPTCAST_RO(&numi1), sizeof(numi1));
          cfg->connected = TRUE;
    	}
    }


#ifdef _WIN32
  sizeinit = strlen("Init");
  vp_com_write_socket(&cfg->socket,"Init",&sizeinit);
#endif
  return res;
}

void video_com_stage_notify_timeout (void)
{
  vp_os_mutex_lock (&registerMutex);
  int index = 0;
  for (index = 0; index < nbRegisteredStages; index ++)
    {
      registeredStages[index]->mustReconnect = 1;
    }
  vp_os_mutex_unlock (&registerMutex);
}

C_RESULT video_com_stage_open(video_com_config_t *cfg)
{
  cfg->connected = FALSE;
  cfg->mustReconnect = 0;
  video_com_stage_register (cfg);
  PDBG ("Will call connect");
  return video_com_stage_connect (cfg);
}

C_RESULT video_com_multisocket_stage_open(video_com_multisocket_config_t *cfg)
{
  int i;
  C_RESULT res = C_OK;
  int nb_failed_connections = 0;
  for (i = 0; i < cfg->nb_sockets; i++)
    {
      video_com_stage_register (cfg->configs[i]);
      cfg->configs[i]->mustReconnect = 0;
    }

  PRINT("Video multisocket : init %i sockets\n",cfg->nb_sockets);

  for(i=0;i<cfg->nb_sockets;i++)
    {
      PRINT("Video multisocket : connecting socket %d on port %d %s\n",
             i,
             cfg->configs[i]->socket.port,
             (cfg->configs[i]->protocol==VP_COM_TCP)?"TCP":"UDP");

      cfg->configs[i]->connected = FALSE;
      res = video_com_stage_connect(cfg->configs[i]);
      if (!VP_SUCCEEDED(res)) {
        PRINT(" - Connection failed\n");
        nb_failed_connections++;

      }
    }

  cfg->last_active_socket = -1;

  if (nb_failed_connections==cfg->nb_sockets) { return C_FAIL; }


  return C_OK;
}



C_RESULT video_com_stage_transform(video_com_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  C_RESULT res;
  vp_os_mutex_lock(&out->lock);
  
  if (1 == cfg->mustReconnect)
    {
      PDBG ("Will call connect");
      PRINT ("Reconnecting ... ");
      res = video_com_stage_connect (cfg);
      PRINT ("%s\n", (VP_FAILED (res) ? "FAIL" : "OK"));
      cfg->mustReconnect = 0;
    }

  if(out->status == VP_API_STATUS_INIT)
    {
      out->numBuffers = 1;
      out->size = 0;
      out->buffers = (uint8_t **) vp_os_malloc (sizeof(uint8_t *)+cfg->buffer_size*sizeof(uint8_t));
      out->buffers[0] = (uint8_t *)(out->buffers+1);
      out->indexBuffer = 0;
      // out->lineSize not used

      out->status = VP_API_STATUS_PROCESSING;
    }

  if(out->status == VP_API_STATUS_PROCESSING && cfg->read != NULL)
    {
      bool_t nonBlock = (cfg->forceNonBlocking && *(cfg->forceNonBlocking)==TRUE) ? TRUE : FALSE;
      out->size = cfg->buffer_size;
      if (nonBlock)
        {
          cfg->socket.block = VP_COM_DONTWAIT;
        }
      res = cfg->read(&cfg->socket, out->buffers[0], &out->size);

      if (! nonBlock && cfg->protocol == VP_COM_UDP && out->size == 0)
        {
          // Send "1" for Unicast
          // Send "2" to enable Multicast
          char flag = 1; int32_t len = sizeof(flag);
          if ( cfg->socket.is_multicast == 1 )
            {
              flag = 2;
            }
          cfg->write(&cfg->socket, (uint8_t*) &flag, &len);
        }

      bool_t bContinue = TRUE;

      if( VP_FAILED(res) )
        {
	  PDBG ("%s [%d] : status set to error !\n", __FUNCTION__, __LINE__);
	  perror ("Video_com_stage");
	  cfg->mustReconnect = 1;
          out->size = 0;
	  vp_os_mutex_unlock (&out->lock);      
	  return C_OK;
        }

      if( out->size == 0)
        {
          if (nonBlock)
            {
              out->size = -1; // Signal next stage that we don't have data waiting
            }
          else
            {
              cfg->num_retries++;
            }
          bContinue = FALSE;
        }
      else
        {
          cfg->num_retries = 0;
        }
 
      cfg->socket.block = VP_COM_DONTWAIT;
      int32_t readSize = cfg->buffer_size - out->size;
      while (TRUE == bContinue)
        {
          res = cfg->read(&cfg->socket, &(out->buffers[0][out->size]), &readSize);
          if( VP_FAILED(res) )
            {
	      PDBG ("%s [%d] : status set to error !\n", __FUNCTION__, __LINE__);
	      perror ("Video_com_stage");
	      cfg->mustReconnect = 1;
              out->size = 0;
	      vp_os_mutex_unlock (&out->lock);	      
	      return C_OK;
            }
          if (0 == readSize)
            {
              bContinue = FALSE;
            }
          out->size += readSize;
          readSize = cfg->buffer_size - out->size;
        }
      cfg->socket.block = VP_COM_DEFAULT;
    }

  if (NULL != cfg->timeoutFunc && 0 != cfg->timeoutFuncAfterSec)
    {
      if (cfg->num_retries >= cfg->timeoutFuncAfterSec)
        {
          cfg->timeoutFunc ();
        }
    }


  vp_os_mutex_unlock(&out->lock);

  return C_OK;
}



C_RESULT video_com_multisocket_stage_transform(video_com_multisocket_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  C_RESULT res;
  fd_set rfds;
  int retval;
  int fs,maxfs;
  struct timeval tv;
  int i;
  bool_t selectTimeout;

  vp_os_mutex_lock(&out->lock);

  out->size = 0;


  for (i=0;i<cfg->nb_sockets;i++) {
    if (1 == cfg->configs[i]->mustReconnect)
      {
        PDBG ("Will call connect");
        PRINT ("Reconnecting ... ");
        res = C_FAIL;
        res = video_com_stage_connect (cfg->configs[i]);
        PRINT ("%s\n", (VP_FAILED (res) ? "FAIL" : "OK"));
        cfg->configs[i]->mustReconnect = 0;
    }
  }

  if(out->status == VP_API_STATUS_INIT)
    {
      out->numBuffers = 1;
      out->size = 0;
      out->buffers = (uint8_t **) vp_os_malloc (sizeof(uint8_t *)+cfg->buffer_size*sizeof(uint8_t));
      out->buffers[0] = (uint8_t *)(out->buffers+1);
      out->indexBuffer = 0;
      // out->lineSize not used

      out->status = VP_API_STATUS_PROCESSING;
    }

  if(out->status == VP_API_STATUS_PROCESSING)
    {

      /* Check which socket has data to read */
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      if(cfg->forceNonBlocking && *(cfg->forceNonBlocking)==TRUE) { tv.tv_sec = 0; }

      FD_ZERO(&rfds);
      maxfs=0;
      for (i=0;i<cfg->nb_sockets;i++) {
        if(cfg->configs[i]->connected) {
          fs = (int)cfg->configs[i]->socket.priv;

          if (fs>maxfs) maxfs=fs;
          FD_SET(fs,&rfds);
        }
      }
        
      retval = select(maxfs+1, &rfds, NULL, NULL, &tv);

      /* Read the socket which has available data */
      selectTimeout = FALSE;
      i = -1;
      if (retval>0)
        {
          if (cfg->last_active_socket!=-1)
            {
              i=cfg->last_active_socket;
              fs = (int)cfg->configs[i]->socket.priv;
              if (cfg->configs[i]->read && FD_ISSET(fs, &rfds))
                {
                  out->size = cfg->configs[i]->buffer_size;
                }
              else
                {
                  i = -1;
                }
            }

          if (-1 == i)
            {
              for (i=0;i<cfg->nb_sockets;i++)
                {
                  fs = (int)cfg->configs[i]->socket.priv;
                  if (cfg->configs[i]->read && FD_ISSET(fs, &rfds)) {

                    DEBUG_PRINT_SDK("Video multisocket : found data on port %s %d\n",
                                    (cfg->configs[i]->socket.protocol==VP_COM_TCP)?"TCP":"UDP",cfg->configs[i]->socket.port);

                    cfg->last_active_socket = i;
                    if (VP_COM_TCP == cfg->configs[i]->protocol)
                      {
                        DEBUG_isTcp = 1;
                      }
                    else
                      {
                        DEBUG_isTcp = 0;
                      }
                    out->size = cfg->configs[i]->buffer_size;
                    break;
                  }
                }
                
                if(i == cfg->nb_sockets)
                {
                    PRINT("%s:%d BUG !!!!!", __FUNCTION__, __LINE__);
                    selectTimeout = TRUE;
                }
            }
        }
      else
        {
          DEBUG_PRINT_SDK("%s\n",(retval==0)?"timeout":"error");
          selectTimeout = TRUE;
        }

      if (FALSE == selectTimeout)
        {
       //   DEBUG_PRINT_SDK ("Will read on socket %d\n", i);
          // Actual first time read
          res = cfg->configs[i]->read(&cfg->configs[i]->socket, out->buffers[0], &out->size);
          if (VP_FAILED (res))
            {
	      PDBG ("%s [%d] : status set to error !\n", __FUNCTION__, __LINE__);
	      perror ("Video_com_stage");
	      cfg->configs[i]->mustReconnect = 1;
              out->size = 0;
	      vp_os_mutex_unlock (&out->lock);
	      return C_OK;
            }
      
          // Loop read to empty the socket buffers if needed
          cfg->configs[i]->socket.block = VP_COM_DONTWAIT;
          bool_t bContinue = TRUE;
          int32_t readSize = cfg->configs[i]->buffer_size - out->size;
          while (TRUE == bContinue)
            {
         //     DEBUG_PRINT_SDK ("Will read %d octets from socket %d\n", readSize, i);
              res = cfg->configs[i]->read(&cfg->configs[i]->socket, &(out->buffers[0][out->size]), &readSize);
              if (VP_FAILED (res))
                {
		  PDBG ("%s [%d] : status set to error !\n", __FUNCTION__, __LINE__);
		  perror ("Video_com_stage");
		  cfg->configs[i]->mustReconnect = 1;
                  out->size = 0;
		  vp_os_mutex_unlock (&out->lock);
		  return C_OK;
                }
              if (0 == readSize)
                {
                  bContinue = FALSE;
                }
              out->size += readSize;
              readSize = cfg->configs[i]->buffer_size - out->size;
            }
          cfg->configs[i]->socket.block = VP_COM_DEFAULT;
        }

      /* Resend a connection packet on UDP sockets */
      if(!(cfg->forceNonBlocking && *(cfg->forceNonBlocking)==TRUE))
      {
      if( /* select timed out */ retval==0 )
        {
          for (i=0;i<cfg->nb_sockets;i++)
            {
              if( cfg->configs[i]->protocol == VP_COM_UDP )
                {
                  // Send "1" for Unicast
                  // Send "2" to enable Multicast
                  char flag = 1;  int len = sizeof(flag);
                  if ( cfg->configs[i]->socket.is_multicast == 1 )
                    flag = 2;

                  DEBUG_PRINT_SDK("Video multisocket : sending connection byte on port %s %d\n",
                                  (cfg->configs[i]->socket.protocol==VP_COM_TCP)?"TCP":"UDP",cfg->configs[i]->socket.port);

                  cfg->configs[i]->write(&cfg->configs[i]->socket, (uint8_t*) &flag, &len);
                }
            }
        }
      }

      if( (TRUE == selectTimeout || out->size == 0) && (!cfg->forceNonBlocking || (*(cfg->forceNonBlocking) == FALSE)) )
      {
          cfg->num_retries++;
      }
      else
          cfg->num_retries = 0;

      if((selectTimeout == TRUE) && cfg->forceNonBlocking && *(cfg->forceNonBlocking)==TRUE)
      {
    	  	 //PRINT("Debug %s:%d\n",__FUNCTION__,__LINE__);

    	  	 /* No data are available here, but some are available in the next stage */
    	     /* out->size=0 would restart the pipeline */
    	  	 out->size=-1;
      }
    }

  vp_os_mutex_unlock(&out->lock);

  if (out->size > 0)
  {
	  DEBUG_totalBytes += out->size;
	  static struct timeval DEBUG_now = {0, 0}, DEBUG_prev = {0, 0};
	  gettimeofday (&DEBUG_now, NULL);
	  float DEBUG_timeDiff = ((DEBUG_now.tv_sec - DEBUG_prev.tv_sec) * 1000.0) + ((DEBUG_now.tv_usec - DEBUG_prev.tv_usec) / 1000.0);
	  if (DEBUG_TIME_BITRATE_CALCULATION_MSEC <= DEBUG_timeDiff)
		{
		  DEBUG_prev.tv_sec  = DEBUG_now.tv_sec;
		  DEBUG_prev.tv_usec = DEBUG_now.tv_usec;
		  float DEBUG_instantBitrate = 8.0 * (float)(DEBUG_totalBytes) / DEBUG_TIME_BITRATE_CALCULATION_MSEC;
		  DEBUG_totalBytes = 0;
		  DEBUG_bitrate = 0.8 * DEBUG_bitrate + 0.2 * DEBUG_instantBitrate;
		}
  }

  return C_OK;
}




C_RESULT video_com_stage_close(video_com_config_t *cfg)
{
  vp_com_close(cfg->com, &cfg->socket);
  return C_OK;
}

C_RESULT video_com_multisocket_stage_close(video_com_multisocket_config_t *cfg)
{
  int i;

  for (i=0;i<cfg->nb_sockets;i++)
    {
      vp_com_close(cfg->configs[i]->com, &cfg->configs[i]->socket);
    }

  return C_OK;
}

