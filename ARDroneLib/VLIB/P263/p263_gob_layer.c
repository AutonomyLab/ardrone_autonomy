#include <VLIB/video_packetizer.h>

#include "p263_codec.h"
#include "p263_layers.h"
#include "p263_huffman.h"

C_RESULT p263_read_gob_layer( video_controller_t* controller, video_stream_t* stream )
{
  p263_codec_t* p263_codec = (p263_codec_t*) controller->video_codec;
  p263_picture_layer_t* picture_layer = &p263_codec->picture_layer;
  p263_gob_layer_t* gob = &picture_layer->gobs[controller->blockline];
  uint32_t gn = 0, gfid = 0, gsbi = 0;

  // Read Group Number (GN) (5 bits)
  video_read_data( stream, &gn, 5 );

  // Read GOB Sub-Bitstream Indicator (GSBI) (2 bits)
  video_read_data( stream, &gsbi, 2 );

  // Read GOB Frame ID (GFID) (2 bits)
  video_read_data( stream, &gfid, 2 );

  // Read Quantizer Information (GQUANT) (5 bits)
  video_read_data( stream, &gob->gquant, 5 );
  controller->Qp = gob->gquant;

  return C_OK;
}

