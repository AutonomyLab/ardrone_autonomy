#ifndef _BUFFER_TO_PICTURE_H_
#define _BUFFER_TO_PICTURE_H_

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_picture.h>

typedef struct _buffer_to_picture_config_t
{
  vp_api_picture_t* picture;
  int32_t           y_buffer_size;
  int32_t           y_blockline_size;
  int32_t           y_current_size;

  int32_t           num_frames;

  uint8_t*          y_buf_ptr;
#ifdef USE_VIDEO_YUV
  uint8_t*          cr_buf_ptr;
  uint8_t*          cb_buf_ptr;
#endif

  int32_t           cumulated_size;
  uint8_t*          input_ptr;

  int32_t           num_picture_decoded;

} buffer_to_picture_config_t;

C_RESULT buffer_to_picture_open(buffer_to_picture_config_t *cfg);
C_RESULT buffer_to_picture_transform(buffer_to_picture_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT buffer_to_picture_close(buffer_to_picture_config_t *cfg);

extern const vp_api_stage_funcs_t buffer_to_picture_funcs;

#endif // _BUFFER_TO_PICTURE_H_
