#ifndef _VIDEO_COM_STAGE_H_
#define _VIDEO_COM_STAGE_H_

#include <VP_Api/vp_api.h>
#include <VP_Com/vp_com.h>

#define VIDEO_MAX_RETRIES 5
 
typedef struct _video_com_config_t
{
  vp_com_t*             com;

  vp_com_socket_t       socket;
  VP_COM_SOCKET_OPTIONS sockopt;

  uint32_t              buffer_size;

  // Private Datas
  Read                  read;
  Write                 write;
  uint32_t              num_retries;

  VP_COM_SOCKET_PROTOCOL protocol;
  bool_t connected;
  bool_t mustReconnect;
  bool_t *forceNonBlocking;

  void (*timeoutFunc)(void);
  uint32_t timeoutFuncAfterSec;

} video_com_config_t;

typedef struct _video_com_multisocket_config_t
{
	uint32_t nb_sockets;
	video_com_config_t ** configs;
	uint32_t last_active_socket;
	uint32_t num_retries;
	uint32_t buffer_size;
	bool_t * forceNonBlocking;

}video_com_multisocket_config_t;


extern const vp_api_stage_funcs_t video_com_funcs;
extern const vp_api_stage_funcs_t video_com_multisocket_funcs;

void video_com_stage_notify_timeout (void);

C_RESULT video_com_stage_open(video_com_config_t *cfg);
C_RESULT video_com_stage_transform(video_com_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT video_com_stage_close(video_com_config_t *cfg);

C_RESULT video_com_multisocket_stage_open(video_com_multisocket_config_t *cfg);
C_RESULT video_com_multisocket_stage_transform(video_com_multisocket_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT video_com_multisocket_stage_close(video_com_multisocket_config_t *cfg);

#endif // _VIDEO_COM_STAGE_H_
