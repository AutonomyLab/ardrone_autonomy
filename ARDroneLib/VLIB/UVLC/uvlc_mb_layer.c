#include <VLIB/video_controller.h>
#include <VLIB/video_packetizer.h>
#include <VLIB/Platform/video_utils.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>

#include "uvlc_codec.h"
#include "uvlc_layers.h"
#include "uvlc.h"

#ifndef HAS_UVLC_WRITE_BLOCK

void uvlc_write_block( video_stream_t* const stream, int16_t* data, int32_t num_coeff )
{
  int32_t* zztable;
  int32_t index, code, run;

  zztable = &video_zztable_t81[1];

  // DC coeff
  code = *data;
  video_write_data( stream, code, 10 );
  num_coeff--;

  // AC coeff
  run = 0;
  while( num_coeff > 0 )
  {
    index = *zztable++;
    code = data[index];
    if( code == 0 )
    {
      run ++;
    }
    else
    {
      num_coeff--;
      uvlc_encode( stream, code, run, num_coeff );
      run = 0;
    }
  }
}

#endif // HAS_UVLC_WRITE_BLOCK

#ifndef HAS_UVLC_READ_BLOCK

C_RESULT uvlc_read_block( video_stream_t* stream, int16_t* data, int32_t* num_coeff )
{
  int32_t* zztable;
  int32_t index, code, run, last, nc;

  zztable = &video_zztable_t81[0];

  nc = *num_coeff;

  // DC coeff
  code = run = last = 0;
  video_read_data( stream, (uint32_t*) &code, 10 );
  data[0] = code;

  if( nc > 0 )
  {
    // AC coeff
    while( last == 0 )
    {
      code = run = last = 0;
      uvlc_decode( stream, &run, &code, &last);

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

#endif

C_RESULT uvlc_write_mb_layer( video_stream_t* stream, video_macroblock_t* mb, int32_t num_macro_blocks )
{
  int16_t* data;
  uint32_t code;

  while( num_macro_blocks > 0 )
  {
    video_write_data( stream, mb->azq, 1 );
    if( !mb->azq )
    {
      code  = 0x80;
      code |= (mb->num_coeff_y0 > 1) << 0;
      code |= (mb->num_coeff_y1 > 1) << 1;
      code |= (mb->num_coeff_y2 > 1) << 2;
      code |= (mb->num_coeff_y3 > 1) << 3;
      code |= (mb->num_coeff_cb > 1) << 4;
      code |= (mb->num_coeff_cr > 1) << 5;
      code |= (mb->dquant != 0) << 6;

      video_write_data( stream, code, 8 );

      if( mb->dquant != 0 )
      {
        code = ( mb->dquant < 0 ) ? ~mb->dquant : mb->dquant;
        video_write_data( stream, code, 2 );
      }

      /**************** Block Y0 ****************/
      data = mb->data;
      uvlc_write_block( stream, data, mb->num_coeff_y0 );

      /**************** Block Y1 ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_write_block( stream, data, mb->num_coeff_y1 );

      /**************** Block Y2 ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_write_block( stream, data, mb->num_coeff_y2 );

      /**************** Block Y3 ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_write_block( stream, data, mb->num_coeff_y3 );

      /**************** Block CB ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_write_block( stream, data, mb->num_coeff_cb );

      /**************** Block CR ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_write_block( stream, data, mb->num_coeff_cr );
    }

    mb ++;
    num_macro_blocks --;
  }

  return C_OK;
}

C_RESULT uvlc_read_mb_layer( video_stream_t* stream, video_macroblock_t* mb, int32_t num_macro_blocks )
{
  int16_t* data;
  uint32_t code;

  vp_os_memset( mb->data, 0, num_macro_blocks * 6 * MCU_BLOCK_SIZE * sizeof(int16_t) );
  while( num_macro_blocks > 0 )
  {
    mb->azq = 0;
    video_read_data( stream, (uint32_t*)&mb->azq, 1 );

    if( mb->azq == 0 )
    {
      video_read_data( stream, &code, 8 );

      mb->num_coeff_y0 = (code >> 0) & 1;
      mb->num_coeff_y1 = (code >> 1) & 1;
      mb->num_coeff_y2 = (code >> 2) & 1;
      mb->num_coeff_y3 = (code >> 3) & 1;
      mb->num_coeff_cb = (code >> 4) & 1;
      mb->num_coeff_cr = (code >> 5) & 1;

      mb->dquant = 0;
      if( (code >> 6) & 1 )
      {
        video_read_data( stream, &code, 2 );

        mb->dquant = (code < 2) ? ~code : code;
      }

      /**************** Block Y0 ****************/
      data = mb->data;
      uvlc_read_block( stream, data, &mb->num_coeff_y0 );

      /**************** Block Y1 ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_read_block( stream, data, &mb->num_coeff_y1 );

      /**************** Block Y2 ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_read_block( stream, data, &mb->num_coeff_y2 );

      /**************** Block Y3 ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_read_block( stream, data, &mb->num_coeff_y3 );

      /**************** Block CB ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_read_block( stream, data, &mb->num_coeff_cb );

      /**************** Block CR ****************/
      data += MCU_BLOCK_SIZE;
      uvlc_read_block( stream, data, &mb->num_coeff_cr );

    }

    mb ++;
    num_macro_blocks --;
  }

  return C_OK;
}
