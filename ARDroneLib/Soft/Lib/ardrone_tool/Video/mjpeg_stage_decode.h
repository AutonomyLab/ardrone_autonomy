#ifndef _MJPEG_STAGE_DECODE_H_
#define _MJPEG_STAGE_DECODE_H_

#include <VP_Api/vp_api.h>
#include <MJPEG/mjpeg.h>

typedef struct _mjpeg_stage_decoding_config_t
{
  stream_t          stream;
  mjpeg_t           mjpeg;
  vp_api_picture_t* picture;

  uint32_t          out_buffer_size;

} mjpeg_stage_decoding_config_t;

C_RESULT mjpeg_stage_decoding_open(mjpeg_stage_decoding_config_t *cfg);
C_RESULT mjpeg_stage_decoding_transform(mjpeg_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT mjpeg_stage_decoding_close(mjpeg_stage_decoding_config_t *cfg);

extern uint32_t mjpeg_stage_num_picture_decoded;
extern const vp_api_stage_funcs_t mjpeg_decoding_funcs;

#endif // _MJPEG_STAGE_DECODE_H_
