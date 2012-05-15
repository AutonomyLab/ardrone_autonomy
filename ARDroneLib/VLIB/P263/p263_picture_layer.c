#include <VP_Os/vp_os_malloc.h>

#include <VLIB/video_controller.h>
#include <VLIB/video_packetizer.h>

#include "p263_codec.h"
#include "p263_layers.h"
#include "p263_huffman.h"

C_RESULT p263_read_picture_layer( video_controller_t* controller, video_stream_t* stream )
{
  uint32_t pei = 0;
  p263_codec_t* p263_codec = (p263_codec_t*) controller->video_codec;
  p263_picture_layer_t* picture_layer = &p263_codec->picture_layer;

  p263_codec->mb_types  = &standard_mb_types[0];
  p263_codec->cbpys     = &cbpy_standard[0];

  // Read Temporal Reference (TR) (8 bits)
  picture_layer->tr = 0;
  video_read_data( stream, &picture_layer->tr, 8 );

  // Read Type Information (PTYPE) (Variable Length)
  picture_layer->ptype      = 0;
  picture_layer->plusptype  = 0;
  picture_layer->opptype    = 0;
  video_read_data( stream, &picture_layer->ptype, 8 );

  switch( PICTURE_FORMAT(picture_layer->ptype) )
  {
    case P263_PICTURE_FORMAT_FORBIDDEN:
      break;

    case P263_PICTURE_FORMAT_SUBQCIF:
      video_controller_set_format( controller, 128, 96 );
      goto P263_PICTURE_FORMAT_NOT_EXTENDED;

    case P263_PICTURE_FORMAT_QCIF:
      video_controller_set_format( controller, 176, 144 );
      goto P263_PICTURE_FORMAT_NOT_EXTENDED;

    case P263_PICTURE_FORMAT_CIF:
      video_controller_set_format( controller, 352, 288 );
      goto P263_PICTURE_FORMAT_NOT_EXTENDED;

    case P263_PICTURE_FORMAT_4QCIF:
      video_controller_set_format( controller, 704, 576 );
      goto P263_PICTURE_FORMAT_NOT_EXTENDED;

    case P263_PICTURE_FORMAT_16CIF:
      video_controller_set_format( controller, 1408, 1152 );

    P263_PICTURE_FORMAT_NOT_EXTENDED:
      video_read_data( stream, &picture_layer->ptype, 5 );
      video_controller_set_picture_type( controller, PICTURE_TYPE(picture_layer->ptype) );
      break;

    case P263_PICTURE_FORMAT_RESERVED:
      break;

    case P263_PICTURE_FORMAT_EXTENDED:
      // Read Plus PTYPE (PLUSPTYPE) (Variable Length) -- Optionnal, see PTYPE
      // Read UFEP
      video_read_data( stream, &picture_layer->plusptype, 3 );
      if( picture_layer->plusptype == 1 )
      {
        // Read OPPTYPE
        video_read_data( stream, &picture_layer->opptype, 18 );
      }

      // Read MPPTYPE
      video_read_data( stream, &picture_layer->plusptype, 9 );
      video_controller_set_picture_type( controller, PICTURE_EXTENDED_TYPE(picture_layer->plusptype) );
      break;
  }

  if( picture_layer->plusptype )
  {
    // Read Continuous Presence Multipoint and Video Multiplex (CPM) (1 bit, see Annex C)
    video_read_data( stream, &picture_layer->cpm, 1 );

    if( picture_layer->cpm )
    {
      // Read Picture Sub-Bitstream Indicator (PSBI) (2 bits)
      video_read_data( stream, &picture_layer->psbi, 2 );
    }
  }

  if( picture_layer->opptype ) // eg UFEP == 001b
  {
    if( HAS_CUSTOM_PICTURE_FORMAT( picture_layer->opptype ) )
    {
      // Read Custom Picture Format (CPFMT) (23 bits)
      video_read_data( stream, &picture_layer->cpfmt, 23 );

      if( (picture_layer->cpfmt >> 19) == 0xF )
      {
        // Read Extended Pixel Aspect Ratio (EPAR) (16 bits)
        video_read_data( stream, &picture_layer->epar, 16 );
      }
    }

    if( HAS_CUSTOM_PCF(picture_layer->opptype) )
    {
      // Read Custom Picture Clock Frequency Code (CPCFC) (8 bits)
      video_read_data( stream, &picture_layer->cpcfc, 8 );

      // Read Extended Temporal Reference (ETR) (2 bits)
      video_read_data( stream, &picture_layer->etr, 2 );
    }

    if( HAS_UNRESTRICTED_MOTION_VECTOR(picture_layer->opptype) )
    {
      // Read Unlimited Unrestricted Motion Vectors Indicator (UUI) (Variable length) -- Optionnal
      video_read_data( stream, &picture_layer->uui, 1 );
      if( picture_layer->uui == 0 )
        video_read_data( stream, &picture_layer->uui, 1 );
    }

    if( HAS_SLICE_STRUCTURED(picture_layer->sss) )
    {
      // Read Slice Structured Submode bits (SSS) (2 bits) -- Optionnal
      video_read_data( stream, &picture_layer->sss, 2 );
    }
  }

  if( controller->picture_type == VIDEO_PICTURE_EP )
  {
    // Read Enhancement Layer Number (ELNUM) (4 bits) -- Optionnal
    video_read_data( stream, &picture_layer->elnum, 4 );

    if( picture_layer->opptype ) // eg UFEP == 001b
    {
      // Read Reference Layer Number (RLNUM) (4 bits) -- Optionnal
      video_read_data( stream, &picture_layer->rlnum, 4 );
    }
  }

  if( HAS_REFERENCE_PICTURE_SELECTION( picture_layer->opptype ) )
  {
    // Read Reference Picture Selection Mode Flags (RPSMF) (3 bits) -- Optionnal
    video_read_data( stream, &picture_layer->rpsmf, 3 );
  }

  if( picture_layer->rpsmf )
  {
    // Read Temporal Reference for Prediction Indication (TRPI) (1 bit) -- Optionnal
    video_read_data( stream, &picture_layer->trpi, 1 );

    if( picture_layer->trpi )
    {
      // Read Temporal Reference for Prediction (TRP) (10 bits) -- Optionnal
      video_read_data( stream, &picture_layer->trp, 10 );
    }

    // Read Back-Channel message Indication (BCI) (Variable length, 1 or 2 bits) -- Optionnal
    video_read_data( stream, &picture_layer->bci, 1 );

    if( picture_layer->bci == 1 )
    {
      // TODO
      // Read Back-Channel Message (BCM) (Variable length, see N.4.2) -- Optionnal
    }
    else
    {
      video_read_data( stream, &picture_layer->bci, 1 ); // read '01' end of back channel message
    }
  }

  if( HAS_REFERENCE_PICTURE_RESAMPLING( picture_layer->plusptype ) )
  {
    // TODO
    // Read Reference Picture Resampling Parameters (RPRP) (Variable length, see Annex P) -- Optionnal
  }

  // Read Quantizer Information (PQUANT) (5 bits)
  video_read_data( stream, &picture_layer->pquant, 5 );
  controller->Qp = picture_layer->pquant;

  if( !picture_layer->plusptype )
  {
    // Read Continuous Presence Multipoint and Video Multiplex (CPM) (1 bit, see Annex C)
    video_read_data( stream, &picture_layer->cpm, 1 );

    if( picture_layer->cpm )
    {
      // Read Picture Sub-Bitstream Indicator (PSBI) (2 bits)
      video_read_data( stream, &picture_layer->psbi, 2 );
    }
  }

  if( controller->picture_type == VIDEO_PICTURE_PB )
  {
    // TODO
    // Read Temporal Reference for B-pictures in PB-frames (TRB) (3/5 bits) -- Optionnal

    // Read Quantization information for B-pictures in PB-frames (DBQUANT) (2 bits)
  }

  // Read Extra Insertion Information (PEI) (1 bit)
  picture_layer->pei = 0;
  video_read_data( stream, &pei, 1 );

  while( pei )
  {
    // Read Supplemental Enhancement Information (PSUPP) (0/8/16 ... bits) -- Optionnal (See Annex L)
    video_read_data( stream, &pei, 8 );

    // picture_layer->psupp[picture_layer->pei++] = pei; TODO: Alloc psupp

    video_read_data( stream, &pei, 1 );
  }

  return C_OK;
}
