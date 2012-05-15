#include <config.h>
#include <VP_Os/vp_os_print.h>

#include <ardrone_tool/Video/mjpeg_stage_decode.h>

uint32_t mjpeg_stage_num_picture_decoded = 0;

const vp_api_stage_funcs_t mjpeg_decoding_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) mjpeg_stage_decoding_open,
  (vp_api_stage_transform_t) mjpeg_stage_decoding_transform,
  (vp_api_stage_close_t) mjpeg_stage_decoding_close
};

C_RESULT mjpeg_stage_decoding_open(mjpeg_stage_decoding_config_t *cfg)
{
  stream_new( &cfg->stream, OUTPUT_STREAM );

  return mjpeg_init( &cfg->mjpeg, MJPEG_DECODE, cfg->picture->width, cfg->picture->height, cfg->picture->format );
}

C_RESULT mjpeg_stage_decoding_transform(mjpeg_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  bool_t got_image;

  vp_os_mutex_lock( &out->lock );

  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers   = 1;
    out->buffers      = (int8_t**) cfg->picture;
    out->indexBuffer  = 0;
    out->lineSize     = 0;

    out->status = VP_API_STATUS_PROCESSING;
  }

  if( in->status == VP_API_STATUS_ENDED ) {
    out->status = in->status;
  }

  // Several cases must be handled in this stage
  // 1st: Input buffer is too small to decode a complete picture
  // 2nd: Input buffer is big enough to decode 1 frame
  // 3rd: Input buffer is so big we can decode more than 1 frame

  if( out->status == VP_API_STATUS_PROCESSING )
  {
    // Reinit stream with new data
    stream_config( &cfg->stream, in->size, in->buffers[in->indexBuffer] );
  }

  if(out->status == VP_API_STATUS_PROCESSING || out->status == VP_API_STATUS_STILL_RUNNING)
  {
    // If out->size == 1 it means picture is ready
    out->size = 0;
    out->status = VP_API_STATUS_PROCESSING;

    mjpeg_decode( &cfg->mjpeg, cfg->picture, &cfg->stream, &got_image );

    if( got_image )
    {
      // we got one picture (handle case 1)
      out->size = 1;

      mjpeg_stage_num_picture_decoded = cfg->mjpeg.num_frames;

#ifndef USE_VIDEO_YUV
      int32_t i;
      for(i = 0; i < cfg->picture->width * cfg->picture->height / 4; i++ )
      {
        cfg->picture->cr_buf[i] = 0x80;
        cfg->picture->cb_buf[i] = 0x80;
      }
#endif

      // handle case 2 & 3
      if( FAILED(stream_is_empty( &cfg->stream )) )
      {
        // Some data are still in stream
        // Next time we run this stage we don't want this data to be lost
        // So flag it!
        out->status = VP_API_STATUS_STILL_RUNNING;
      }
    }
  }

  vp_os_mutex_unlock( &out->lock );

  return C_OK;
}

C_RESULT mjpeg_stage_decoding_close(mjpeg_stage_decoding_config_t *cfg)
{
  stream_delete( &cfg->stream );

  return mjpeg_release( &cfg->mjpeg );
}
