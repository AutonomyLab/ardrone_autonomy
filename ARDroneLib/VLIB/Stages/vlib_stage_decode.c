#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>
#include <VLIB/Stages/vlib_stage_decode.h>


#define FRAME_MODE_BUFFER_SIZE 256

static video_stream_t stream;

const vp_api_stage_funcs_t vlib_decoding_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) vlib_stage_decoding_open,
  (vp_api_stage_transform_t) vlib_stage_decoding_transform,
  (vp_api_stage_close_t) vlib_stage_decoding_close
};

C_RESULT vlib_stage_decoding_open(vlib_stage_decoding_config_t *cfg)
{
  // init video decoder with NULL_CODEC
  video_codec_open( &cfg->controller, NULL_CODEC );

  if(cfg->block_mode_enable)
  {
    vp_os_free( cfg->controller.in_stream.bytes );
    cfg->controller.in_stream.bytes = NULL;
  }
  else
  {
    stream.bytes  = (uint32_t*)vp_os_malloc(FRAME_MODE_BUFFER_SIZE*sizeof(uint32_t));
    stream.index  = 0;
    stream.used   = 0;
    stream.size   = FRAME_MODE_BUFFER_SIZE*sizeof(uint32_t);
  }

  cfg->num_picture_decoded = 0;

  return C_OK;
}

C_RESULT vlib_stage_decoding_transform(vlib_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  bool_t got_image;
  C_RESULT decodeOk = C_FAIL;

  vp_os_mutex_lock( &out->lock );

  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers   = 1;
    out->buffers      = (uint8_t**)&(cfg->picture->y_buf);
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

    cfg->picture->vision_complete = 0;
    cfg->picture->complete        = 0;
    cfg->picture->blockline       = 0;

    got_image = FALSE;

    if(cfg->block_mode_enable)
    {
      cfg->controller.in_stream.bytes   = (uint32_t*)in->buffers[0];
      cfg->controller.in_stream.used    = in->size;
      cfg->controller.in_stream.size    = in->size;
      cfg->controller.in_stream.index   = 0;
      cfg->controller.in_stream.length  = 32;
      cfg->controller.in_stream.code    = 0;

      decodeOk = video_decode_blockline( &cfg->controller, cfg->picture, &got_image );
    }
    else
    {
      stream.index  = 0;
      stream.used   = in->size;
      vp_os_memcpy(&stream.bytes[0], (uint32_t*)in->buffers[0], in->size);

      decodeOk = video_decode_picture( &cfg->controller, cfg->picture, &stream, &got_image );
    }

	  if( got_image &&
          C_OK == decodeOk)
    {
      // we got one picture
      out->size = 1;
      cfg->picture->complete        = 1;
      cfg->num_picture_decoded++;

      if( cfg->luma_only )
      {
        int32_t i;
        for(i = 0; i < (int32_t)cfg->picture->width * (int32_t)cfg->picture->height / 4; i++ )
        {
          cfg->picture->cr_buf[i] = 0x80;
          cfg->picture->cb_buf[i] = 0x80;
        }
      }
    }
  }

  vp_os_mutex_unlock( &out->lock );

  return C_OK;
}

C_RESULT vlib_stage_decoding_close(vlib_stage_decoding_config_t *cfg)
{
  if(!cfg->block_mode_enable)
  {
    vp_os_free(stream.bytes);
    stream.bytes = NULL;
  }
  else
    cfg->controller.in_stream.bytes = NULL;
    
  return video_codec_close( &cfg->controller );
}
