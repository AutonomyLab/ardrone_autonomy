#include <VLIB/video_controller.h>
#include <VLIB/video_packetizer.h>

#include "p264_codec.h"
#include "p264_layers.h"
#include <VP_Os/vp_os_print.h>

C_RESULT p264_write_picture_layer( video_controller_t* controller, video_stream_t* stream )
{
  uint32_t format = 0, resolution = 0, width, height;

  p264_codec_t* p264_codec = (p264_codec_t*) controller->video_codec;
  p264_picture_layer_t* picture_layer = &p264_codec->picture_layer;

  width   = controller->width;
  height  = controller->height;

  while( format == 0 )
  {
    if( width == QQCIF_WIDTH )
      format = UVLC_FORMAT_CIF;

    if( width == QQVGA_WIDTH )
      format = UVLC_FORMAT_VGA;

    width   >>= 1;
    height  >>= 1;

    resolution ++;
  }

  picture_layer->format     = format;
  picture_layer->resolution = resolution;

  video_write_data( stream, picture_layer->format, 2 );
  video_write_data( stream, picture_layer->resolution, 3 );
  video_write_data( stream, picture_layer->picture_type, 3 );
  video_write_data( stream, picture_layer->quant, 6 );
  video_write_data( stream, controller->num_frames, 32 );

  return C_OK;
}

C_RESULT p264_read_picture_layer( video_controller_t* controller, video_stream_t* stream )
{
  uint32_t width, height;

  p264_codec_t* p264_codec = (p264_codec_t*) controller->video_codec;
  p264_picture_layer_t* picture_layer = &p264_codec->picture_layer;

  picture_layer->format       = 0;
  picture_layer->resolution   = 0;
  picture_layer->picture_type = 0;
  picture_layer->quant        = 0;

  video_read_data( stream, &picture_layer->format, 2 );
  video_read_data( stream, &picture_layer->resolution, 3 );
  video_read_data( stream, &picture_layer->picture_type, 3 );
  video_read_data( stream, &picture_layer->quant, 6 );
  video_read_data( stream, &controller->num_frames, 32 );

  switch( picture_layer->format )
  {
    case UVLC_FORMAT_CIF:
      width   = QQCIF_WIDTH << (picture_layer->resolution-1);
      height  = QQCIF_HEIGHT << (picture_layer->resolution-1);
      break;

    case UVLC_FORMAT_VGA:
      width   = QQVGA_WIDTH << (picture_layer->resolution-1);
      height  = QQVGA_HEIGHT << (picture_layer->resolution-1);
      break;

    default:
      width   = 0;
      height  = 0;
      break;
  }

  video_controller_set_format( controller, width, height );

  return C_OK;
}
