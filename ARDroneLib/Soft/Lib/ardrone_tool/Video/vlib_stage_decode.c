#include <config.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

#include <ardrone_tool/Video/vlib_stage_decode.h>

const vp_api_stage_funcs_t vlib_decoding_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) vlib_stage_decoding_open,
  (vp_api_stage_transform_t) vlib_stage_decoding_transform,
  (vp_api_stage_close_t) vlib_stage_decoding_close
};

C_RESULT vlib_stage_decoding_open(vlib_stage_decoding_config_t *cfg)
{
  video_codec_open( &cfg->controller, UVLC_CODEC );
  video_controller_set_motion_estimation( &cfg->controller, FALSE );
  video_controller_set_format( &cfg->controller, ACQ_WIDTH, ACQ_HEIGHT );

  vp_os_free( cfg->controller.in_stream.bytes );

  return C_OK;
}

C_RESULT vlib_stage_decoding_transform(vlib_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  bool_t got_image;

  vp_os_mutex_lock( &out->lock );

  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers   = 1;
    out->buffers      = (int8_t**)&cfg->picture;
    out->indexBuffer  = 0;
    out->lineSize     = 0;

    out->status = VP_API_STATUS_PROCESSING;
  }

  if( in->status == VP_API_STATUS_ENDED ) {
    out->status = in->status;
  }

  if(out->status == VP_API_STATUS_PROCESSING )
  {
    // If out->size == 1 it means picture is ready
    out->size = 0;

    cfg->controller.in_stream.bytes   = (uint32_t*)in->buffers[0];
    cfg->controller.in_stream.used    = in->size;
    cfg->controller.in_stream.size    = in->size;
    cfg->controller.in_stream.index   = 0;
    cfg->controller.in_stream.length  = 32;
    cfg->controller.in_stream.code    = 0;

    got_image = FALSE;
    video_decode_blockline( &cfg->controller, cfg->picture, &got_image );

    if( got_image )
    {
      // we got one picture
      out->size = 1;

#ifndef USE_VIDEO_YUV
      int32_t i;
      for(i = 0; i < cfg->picture->width * cfg->picture->height / 4; i++ )
      {
        cfg->picture->cr_buf[i] = 0x80;
        cfg->picture->cb_buf[i] = 0x80;
      }
#endif
    }
  }

  vp_os_mutex_unlock( &out->lock );

  return C_OK;
}

C_RESULT vlib_stage_decoding_close(vlib_stage_decoding_config_t *cfg)
{
  return video_codec_close( &cfg->controller );
}
