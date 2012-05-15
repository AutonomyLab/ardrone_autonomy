
#ifndef _BUFFER_TO_PICTURE_H_
#define _BUFFER_TO_PICTURE_H_


#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_picture.h>

typedef void (*custom_data_handler_cb)(void* data, int32_t custom_data_size);

typedef struct _vp_stages_buffer_to_picture_config_t
{
  vp_api_picture_t*       picture;
  int32_t                 y_buffer_size;
  int32_t                 y_blockline_size;
  int32_t                 y_current_size;

  int32_t                 num_frames;

  uint8_t*                y_buf_ptr;
  uint8_t*                cr_buf_ptr;
  uint8_t*                cb_buf_ptr;

  bool_t                  luma_only;
  bool_t                  block_mode_enable;

  int32_t                 cumulated_size;
  uint8_t*                input_ptr;

  int32_t                 num_picture_decoded;

  uint32_t                custom_data_size;
  uint32_t                custom_data_read;
  uint8_t*                custom_data_ptr;
  custom_data_handler_cb  custom_data_handler;

} vp_stages_buffer_to_picture_config_t;

typedef struct _vp_stages_picture_to_buffer_config_t
{
  vp_api_picture_t*       picture;
  int32_t                 y_buffer_size;
  int32_t                 y_blockline_size;

  bool_t                  luma_only;
  bool_t                  block_mode_enable;

  int32_t                 num_picture_encoded;

  uint32_t                custom_data_size;
  custom_data_handler_cb  custom_data_handler;

} vp_stages_picture_to_buffer_config_t;


C_RESULT vp_stages_buffer_to_picture_open(vp_stages_buffer_to_picture_config_t *cfg);
C_RESULT vp_stages_buffer_to_picture_transform(vp_stages_buffer_to_picture_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT vp_stages_buffer_to_picture_close(vp_stages_buffer_to_picture_config_t *cfg);


C_RESULT vp_stages_picture_to_buffer_open(vp_stages_picture_to_buffer_config_t *cfg);
C_RESULT vp_stages_picture_to_buffer_transform(vp_stages_picture_to_buffer_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT vp_stages_picture_to_buffer_close(vp_stages_picture_to_buffer_config_t *cfg);

#endif // _BUFFER_TO_PICTURE_H_

