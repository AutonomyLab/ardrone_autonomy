#include "video.h"

#define NB_STAGES 10
#define CAMIF_H_CAMERA_USED CAMIF_CAMERA_OVTRULY

unsigned char buffer[STREAM_WIDTH * STREAM_HEIGHT * 3];
int current_frame_id = 0;

extern "C" C_RESULT export_stage_open( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	return (SUCCESS);
}

extern "C" C_RESULT export_stage_transform( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	memcpy(buffer, in->buffers[0], STREAM_WIDTH * STREAM_HEIGHT * 3);
	current_frame_id++;
 	return (SUCCESS);
}

extern "C" C_RESULT export_stage_close( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	return (SUCCESS);
}

const vp_api_stage_funcs_t vp_stages_export_funcs =
{
	NULL,
	(vp_api_stage_open_t)export_stage_open,
	(vp_api_stage_transform_t)export_stage_transform,
	(vp_api_stage_close_t)export_stage_close
};

DEFINE_THREAD_ROUTINE(video_update_thread, data)
{
  C_RESULT res;

  vp_api_io_pipeline_t    pipeline;
  vp_api_io_data_t        out;
  vp_api_io_stage_t       stages[NB_STAGES];

  vp_api_picture_t picture;

  video_com_config_t              icc;
  vlib_stage_decoding_config_t    vec;
  vp_stages_yuv2rgb_config_t      yuv2rgbconf;

  /// Picture configuration
  picture.format        = PIX_FMT_YUV420P;
  picture.width         = STREAM_WIDTH;
  picture.height        = STREAM_HEIGHT;
  picture.framerate     = 30;

  picture.y_buf   = (uint8_t *)vp_os_malloc( STREAM_WIDTH * STREAM_HEIGHT     );
  picture.cr_buf  = (uint8_t *)vp_os_malloc( STREAM_WIDTH * STREAM_HEIGHT / 4 );
  picture.cb_buf  = (uint8_t *)vp_os_malloc( STREAM_WIDTH * STREAM_HEIGHT / 4 );

  picture.y_line_size   = STREAM_WIDTH;
  picture.cb_line_size  = STREAM_WIDTH / 2;
  picture.cr_line_size  = STREAM_WIDTH / 2;

  vp_os_memset(&icc,          0, sizeof( icc ));
  vp_os_memset(&vec,          0, sizeof( vec ));
  vp_os_memset(&yuv2rgbconf,  0, sizeof( yuv2rgbconf ));

  icc.com                 = COM_VIDEO();
  icc.buffer_size         = 100000;
  icc.protocol            = VP_COM_UDP;
  COM_CONFIG_SOCKET_VIDEO(&icc.socket, VP_COM_CLIENT, VIDEO_PORT, wifi_ardrone_ip);

  vec.width               = STREAM_WIDTH;
  vec.height              = STREAM_HEIGHT;
  vec.picture             = &picture;
#ifdef USE_VIDEO_YUV
  vec.luma_only           = FALSE;
#else
  vec.luma_only           = TRUE;
#endif // USE_VIDEO_YUV
  vec.block_mode_enable   = TRUE;

  vec.luma_only           = FALSE;
  yuv2rgbconf.rgb_format = VP_STAGES_RGB_FORMAT_RGB24;
  if( CAMIF_H_CAMERA_USED == CAMIF_CAMERA_OVTRULY_UPSIDE_DOWN_ONE_BLOCKLINE_LESS )
    yuv2rgbconf.mode = VP_STAGES_YUV2RGB_MODE_UPSIDE_DOWN;

  pipeline.nb_stages = 0;

  stages[pipeline.nb_stages].type    = VP_API_INPUT_SOCKET;
  stages[pipeline.nb_stages].cfg     = (void *)&icc;
  stages[pipeline.nb_stages].funcs   = video_com_funcs;

  pipeline.nb_stages++;

  stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
  stages[pipeline.nb_stages].cfg     = (void*)&vec;
  stages[pipeline.nb_stages].funcs   = vlib_decoding_funcs;

  pipeline.nb_stages++;

  stages[pipeline.nb_stages].type    = VP_API_FILTER_YUV2RGB;
  stages[pipeline.nb_stages].cfg     = (void*)&yuv2rgbconf;
  stages[pipeline.nb_stages].funcs   = vp_stages_yuv2rgb_funcs;

  pipeline.nb_stages++;

  stages[pipeline.nb_stages].type    = VP_API_OUTPUT_SDL;
  stages[pipeline.nb_stages].cfg     = (void *)&vec;
  stages[pipeline.nb_stages].funcs   = vp_stages_export_funcs;

  pipeline.nb_stages++;

  pipeline.stages = &stages[0];
  PIPELINE_HANDLE pipeline_handle;
  res = vp_api_open(&pipeline, &pipeline_handle);

  if( SUCCEED(res) ) {
    while( SUCCEED(vp_api_run(&pipeline, &out)) ) {
    }

    vp_api_close(&pipeline, &pipeline_handle);
  }

  return (THREAD_RET)0;
}

