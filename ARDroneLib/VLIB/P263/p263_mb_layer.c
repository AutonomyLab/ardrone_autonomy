#include <VP_Os/vp_os_assert.h>
#include <VLIB/video_packetizer.h>

#include "p263_codec.h"
#include "p263_layers.h"
#include "p263_huffman.h"

p263_mb_type_t standard_mb_types[STANDARD_MB_TYPES_NUM] = {
  MAKE_MB_TYPE( 1, 1, 1, 0, 1, 0 ), // INTER
  MAKE_MB_TYPE( 1, 1, 1, 1, 1, 0 ), // INTER+Q
  MAKE_MB_TYPE( 1, 1, 1, 0, 1, 1 ), // INTER4V
  MAKE_MB_TYPE( 1, 1, 1, 0, 0, 0 ), // INTRA
  MAKE_MB_TYPE( 1, 1, 1, 1, 0, 0 ), // INTRA+Q
  MAKE_MB_TYPE( 1, 1, 1, 1, 1, 1 ), // INTER4V+Q
  MAKE_MB_TYPE( 1, 1, 0, 0, 0, 0 )  // Stuffing
};

static C_RESULT p263_read_block( video_stream_t* stream, int16_t* data, int32_t* num_coeff )
{
  int32_t* zztable;
  int32_t index, code, run, last, nc, idx, sign;
  p263_tcoeff_t* tc;

  zztable = &video_zztable_t81[0];

  nc = *num_coeff;

  // DC coeff
  code = run = last = 0;
  video_read_data( stream, (uint32_t*) &code, 8 );
  data[0] = code;

  if( nc > 0 )
  {
    // AC coeff
    while( last == 0 )
    {
      idx = huffman_stream_code( vlc_tcoeff_tree, stream );
      VP_OS_ASSERT(idx < 0 );

      sign = 0;
      tc    = &tcoeff[idx];
      if( tc->last == VLC_TCOEFF_ESCAPE )
      {
        code = run = last = 0;

        video_read_data( stream, (uint32_t*) &last, 1 );
        video_read_data( stream, (uint32_t*) &run, 6 );
        video_read_data( stream, (uint32_t*) &code, 8 );

        // For level (variable code in this program) the code 0000 0000 is forbidden,
        // and the code 1000 0000 is forbidden unless the Modified Quantization mode is
        // in use (see Annex T).

        // Do sign extension for code. see Table 17/H.263 – FLC table for RUNs and LEVELs
        code <<= 24;
        code >>= 24; // This works because shift is signed & code can't be zero 
      }
      else
      {
        // read sign
        video_read_data( stream, (uint32_t*) &sign, 1 );
      }

      code  = (sign == 0 ) ? tc->level : -tc->level;
      run   = tc->run;
      last  = tc->last;

      VP_OS_ASSERT( run < 64 );

      if( last == 0 )
      {
        zztable    += (run+1);
        index       = *zztable;
        data[index] = code;

        nc++;
      }
    }

    *num_coeff = nc;
  }

  return C_OK;
}

C_RESULT p263_read_mb_layer( video_controller_t* controller, video_stream_t* stream, video_macroblock_t* mb )
{
  C_RESULT res = C_OK;
  uint32_t cod = 0;
  p263_mcbpc_t* mcbpc = NULL;
  p263_cbpy_t   cbpy;
  p263_codec_t* p263_codec = (p263_codec_t*) controller->video_codec;
  p263_mb_type_t mb_type = MAKE_MB_TYPE( 1, 0, 0, 0, 0, 0 ); // By default we have only cod
  int32_t dquant = 0, idx = 0;
  int16_t* data;

  if( controller->picture_type != VIDEO_PICTURE_INTRA )
  {
    // Read Coded macroblock indication (COD) (1 bit)
    video_read_data( stream, &cod, 1 );
  }

  if( cod == 0 ) // Macroblock is coded (see 5.3.1)
  {
    // Read Macroblock type & Coded Block Pattern for Chrominance (MCBPC) (Variable length)
    idx     = huffman_stream_code( vlc_mcbpc_ipictures_tree, stream );
    mcbpc   = &mcbpc_ipictures[idx];

    mb->num_coeff_cb = CBPC_CB(*mcbpc);
    mb->num_coeff_cr = CBPC_CR(*mcbpc);

    mb_type = p263_codec->mb_types[mcbpc->mb_type];
  }

  if( MB_TYPE_HAS_CBPY(mb_type) )
  {
    // Coded Block Pattern for luminance (CBPY) (Variable length)
    idx = huffman_stream_code( vlc_cbpy_standard_tree, stream );
    cbpy = p263_codec->cbpys[idx];

    switch( controller->picture_type )
    {
      case VIDEO_PICTURE_INTRA:
        mb->num_coeff_y0 = CBPY_INTRA_Y0(cbpy);
        mb->num_coeff_y1 = CBPY_INTRA_Y1(cbpy);
        mb->num_coeff_y2 = CBPY_INTRA_Y2(cbpy);
        mb->num_coeff_y3 = CBPY_INTRA_Y3(cbpy);
        break;

      case VIDEO_PICTURE_INTER:
        mb->num_coeff_y0 = CBPY_INTER_Y0(cbpy);
        mb->num_coeff_y1 = CBPY_INTER_Y1(cbpy);
        mb->num_coeff_y2 = CBPY_INTER_Y2(cbpy);
        mb->num_coeff_y3 = CBPY_INTER_Y3(cbpy);
        break;

      default:
        // Picture type not supported
        break;
    }
  }

  if( MB_TYPE_HAS_DQUANT(mb_type) )
  {
    mb->dquant = 0;
    video_read_data( stream, (uint32_t*) &mb->dquant, 2 );

    dquant = ( mb->dquant < 2 ) ? ~mb->dquant : (mb->dquant - 1);
  }

  dquant += controller->Qp;
  mb->dquant = dquant;

  /**************** Block Y0 ****************/
  data = mb->data;
  p263_read_block( stream, data, &mb->num_coeff_y0 );

  /**************** Block Y1 ****************/
  data += MCU_BLOCK_SIZE;
  p263_read_block( stream, data, &mb->num_coeff_y1 );

  /**************** Block Y2 ****************/
  data += MCU_BLOCK_SIZE;
  p263_read_block( stream, data, &mb->num_coeff_y2 );

  /**************** Block Y3 ****************/
  data += MCU_BLOCK_SIZE;
  p263_read_block( stream, data, &mb->num_coeff_y3 );

  /**************** Block CB ****************/
  data += MCU_BLOCK_SIZE;
  p263_read_block( stream, data, &mb->num_coeff_cb );

  /**************** Block CR ****************/
  data += MCU_BLOCK_SIZE;
  p263_read_block( stream, data, &mb->num_coeff_cr );

  return res;
}
