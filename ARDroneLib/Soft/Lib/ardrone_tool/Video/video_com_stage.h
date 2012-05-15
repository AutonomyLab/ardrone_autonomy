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

} video_com_config_t;

extern const vp_api_stage_funcs_t video_com_funcs;

C_RESULT video_com_stage_open(video_com_config_t *cfg);
C_RESULT video_com_stage_transform(video_com_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT video_com_stage_close(video_com_config_t *cfg);

#endif // _VIDEO_COM_STAGE_H_
