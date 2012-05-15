#include <config.h>

#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>

#include <ardrone_tool/Video/video_com_stage.h>

#include <VP_Com/vp_com_socket.h>

#ifndef _WIN32
	#include <sys/socket.h>
	#include <sys/ioctl.h>
	#include <netinet/in.h>
	#include <netinet/tcp.h>
	#include <unistd.h>
	#define SSOPTCAST_RW(x) x
	#define SSOPTCAST_RO(x) x
#else
	#define SSOPTCAST_RO(x)  (const char*)x
	#define SSOPTCAST_RW(x)  (char*)x
	typedef int socklen_t;
#endif

const vp_api_stage_funcs_t video_com_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) video_com_stage_open,
  (vp_api_stage_transform_t) video_com_stage_transform,
  (vp_api_stage_close_t) video_com_stage_close
};

C_RESULT video_com_stage_open(video_com_config_t *cfg)
{
#ifdef _WIN32
	int sizeinit;
#endif

  C_RESULT res = C_FAIL;

  if( cfg->protocol == VP_COM_UDP )
  {
    struct timeval tv;
#ifdef _WIN32
    int timeout_for_windows=1000; /* timeout in milliseconds */
#endif

    // 1 second timeout
    tv.tv_sec   = 1;
    tv.tv_usec  = 0;

    cfg->socket.protocol = VP_COM_UDP;
    cfg->socket.is_multicast = 1; // enable multicast for video
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
      numi= 256*256;
      setsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVBUF, SSOPTCAST_RO(&numi),numi1);
      getsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_RCVBUF, SSOPTCAST_RW(&numi),&numi1);
		
      numi1=0;
      setsockopt( (int32_t)cfg->socket.priv, SOL_SOCKET, SO_DEBUG, SSOPTCAST_RO(&numi1), sizeof(numi1));
    }
  }
  else if( cfg->protocol == VP_COM_TCP )
  {
    res = vp_com_open(cfg->com, &cfg->socket, &cfg->read, &cfg->write);

    if( VP_SUCCEEDED(res) )
    {
      vp_com_sockopt(cfg->com, &cfg->socket, cfg->sockopt);
    }
  }

  /* Stephane */
	#ifdef _WIN32
	  sizeinit = strlen("Init");
	  vp_com_write_socket(&cfg->socket,"Init",&sizeinit);
	#endif
	return res;
}

C_RESULT video_com_stage_transform(video_com_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  C_RESULT res;
  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers = 1;
    out->size = cfg->buffer_size;
    out->buffers = (int8_t **) vp_os_malloc (sizeof(int8_t *)+out->size*sizeof(int8_t));
    out->buffers[0] = (int8_t *)(out->buffers+1);
    out->indexBuffer = 0;
    // out->lineSize not used

    out->status = VP_API_STATUS_PROCESSING;
  }

  if(out->status == VP_API_STATUS_PROCESSING && cfg->read != NULL)
  {
    out->size = cfg->buffer_size;
    res = cfg->read(&cfg->socket, out->buffers[0], &out->size);

    if( cfg->protocol == VP_COM_UDP )
    {
      if( out->size == 0 )
      {
        // Send "1" for Unicast
        // Send "2" to enable Multicast
        int32_t flag = 1, len = sizeof(flag);
        if ( cfg->socket.is_multicast == 1 )
          flag = 2;

        cfg->write(&cfg->socket, (int8_t*) &flag, &len);
      }
    }

    if( VP_FAILED(res) )
    {
      out->status = VP_API_STATUS_ERROR;
      out->size = 0;
    }

    if( out->size == 0)
       cfg->num_retries++;
    else
       cfg->num_retries = 0;
  }

  vp_os_mutex_unlock(&out->lock);

  return C_OK;
}


C_RESULT video_com_stage_close(video_com_config_t *cfg)
{
  vp_com_close(cfg->com, &cfg->socket);
  return C_OK;
}

