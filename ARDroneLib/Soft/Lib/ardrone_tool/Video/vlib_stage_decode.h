#ifndef _VLIB_STAGE_DECODE_H_
#define _VLIB_STAGE_DECODE_H_

#include <VP_Api/vp_api.h>
#include <VLIB/video_codec.h>

typedef struct _vlib_stage_decoding_config_t
{
  video_controller_t  controller;
  vp_api_picture_t*   picture;

} vlib_stage_decoding_config_t;

C_RESULT vlib_stage_decoding_open(vlib_stage_decoding_config_t *cfg);
C_RESULT vlib_stage_decoding_transform(vlib_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT vlib_stage_decoding_close(vlib_stage_decoding_config_t *cfg);

extern const vp_api_stage_funcs_t vlib_decoding_funcs;

#endif // _VLIB_STAGE_DECODE_H_
