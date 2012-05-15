/**
 *  \brief    vlib Stage
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     first release 06/12/2007
 */
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

#include <VLIB/Stages/vlib_stage_encode.h>
#include <VLIB/video_packetizer.h>

///*******************************************************************************************************************///

const vp_api_stage_funcs_t vlib_encoding_funcs = {
  (vp_api_stage_handle_msg_t) NULL,
  (vp_api_stage_open_t) vlib_stage_encoding_open,
  (vp_api_stage_transform_t) vlib_stage_encoding_transform,
  (vp_api_stage_close_t) vlib_stage_encoding_close
};

///*******************************************************************************************************************///

C_RESULT vlib_stage_encoding_open(vlib_stage_encoding_config_t *cfg)
{
  video_codec_open( &cfg->controller, cfg->codec_type );
  video_controller_set_mode( &cfg->controller, VIDEO_ENCODE );
  video_controller_set_format( &cfg->controller, cfg->width, cfg->height );
  video_controller_set_motion_estimation( &cfg->controller, FALSE );
  video_controller_set_target_size( &cfg->controller, cfg->target_size );
  return C_OK;
}

C_RESULT vlib_stage_encoding_transform(vlib_stage_encoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  static int32_t local_subsampl = 0;

  vp_os_mutex_lock(&out->lock);

  if( out->status == VP_API_STATUS_INIT )
  {
    out->numBuffers   = 1;
    out->buffers      = (int8_t**)(int8_t*)&cfg->controller.in_stream.bytes;
    out->indexBuffer  = 0;

    out->status = VP_API_STATUS_PROCESSING;

    cfg->current_size = 0;
  }

  if( local_subsampl == 0 && out->status == VP_API_STATUS_PROCESSING )
  {
    // check video_codec didn't change
    if (cfg->controller.codec_type != cfg->codec_type)
    {
      video_codec_open( &cfg->controller, cfg->codec_type );
      video_controller_set_mode( &cfg->controller, VIDEO_ENCODE );
      video_controller_set_format( &cfg->controller, cfg->width, cfg->height );
      video_controller_set_motion_estimation( &cfg->controller, FALSE );
    }

    // update target size
    video_controller_set_target_size( &cfg->controller, cfg->target_size );
    RTMON_USTART(VIDEO_VLIB_ENCODE_EVENT);
    if(cfg->block_mode_enable)
      video_encode_blockline( &cfg->controller, cfg->picture, cfg->picture->complete );
    else
      video_encode_picture( &cfg->controller, cfg->picture, (bool_t*)&cfg->picture->complete );
    RTMON_USTOP(VIDEO_VLIB_ENCODE_EVENT);

    if(cfg->picture->complete)
    {
      RTMON_UVAL(ENCODED_PICTURE_UVAL, cfg->controller.num_frames);
      local_subsampl++;
    }

    cfg->current_size = cfg->controller.in_stream.used;

    if( cfg->controller.in_stream.length != 32 )
    {
      // flush & reset internal stream
      video_write_data( &cfg->controller.in_stream, 0, cfg->controller.in_stream.length+1 );
      cfg->controller.in_stream.length = 32;
    }
    out->size = cfg->controller.in_stream.used;

    RTMON_UVAL(ENCODED_BLOCKLINE_SIZE_UVAL, out->size);

    cfg->controller.in_stream.used  = 0;
    cfg->controller.in_stream.index = 0;
  }
  else
  {
    out->size = 0;

    if( cfg->picture->complete )
    {
      local_subsampl++;
    }
  }

  if(local_subsampl >= (int32_t)cfg->subsampl)
    local_subsampl = 0;
  vp_os_mutex_unlock( &out->lock );

  return C_OK;
}

C_RESULT vlib_stage_encoding_close(vlib_stage_encoding_config_t *cfg)
{
  return video_codec_close( &cfg->controller );
}


///*******************************************************************************************************************///
