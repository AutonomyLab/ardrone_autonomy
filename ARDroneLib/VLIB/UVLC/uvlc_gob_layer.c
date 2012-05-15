#include <VLIB/video_controller.h>
#include <VLIB/video_packetizer.h>

#include "uvlc_codec.h"
#include "uvlc_layers.h"

C_RESULT uvlc_write_gob_layer( video_stream_t* stream, uvlc_gob_layer_t* gob )
{
  video_write_data( stream, gob->quant, 5 );

  return C_OK;
}

C_RESULT uvlc_read_gob_layer( video_stream_t* stream, uvlc_gob_layer_t* gob )
{
  gob->quant = 0;

  video_read_data( stream, &gob->quant, 5 );

  return C_OK;
}
