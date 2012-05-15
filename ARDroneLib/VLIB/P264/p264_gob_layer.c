#include <VLIB/video_controller.h>
#include <VLIB/video_packetizer.h>

#include "p264_codec.h"
#include "p264_layers.h"

C_RESULT p264_write_gob_layer( video_stream_t* stream, p264_gob_layer_t* gob )
{
  video_write_data( stream, gob->quant, 6 );

  return C_OK;
}

C_RESULT p264_read_gob_layer( video_stream_t* stream, p264_gob_layer_t* gob )
{
  gob->quant = 0;

  video_read_data( stream, &gob->quant, 6 );

  return C_OK;
}
