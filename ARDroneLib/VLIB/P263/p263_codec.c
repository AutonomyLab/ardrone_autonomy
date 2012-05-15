#include <VLIB/Platform/video_utils.h>
#include <VLIB/Platform/video_config.h>

#include <VLIB/video_quantizer.h>
#include <VLIB/video_dct.h>
#include <VLIB/video_packetizer.h>
#include <VLIB/video_mem32.h>
#include "p263_codec.h"

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>

static int32_t first_init = 1;

const p263_codec_t p263_codec = {
  p263_encode_blockline,
  p263_decode_blockline,
  p263_update,
  p263_cache,
  { 0 }
};

void p263_codec_alloc( video_controller_t* controller )
{
  video_codec_t* video_codec;

  if( first_init == 1 )
  {
    vlc_mcbpc_ipictures_tree = huffman_alloc(VLC_MCBPC_IPICTURES_NUM, VLC_MCBPC_IPICTURES_MAX_LENGTH);
    huffman_add_codes( vlc_mcbpc_ipictures_tree, &vlc_mcbpc_ipictures[0], VLC_MCBPC_IPICTURES_NUM );
    huffman_sort_codes( vlc_mcbpc_ipictures_tree );

    vlc_cbpy_standard_tree = huffman_alloc(VLC_CBPY_STANDARD_NUM, VLC_CBPY_STANDARD_MAX_LENGTH);
    huffman_add_codes( vlc_cbpy_standard_tree, &vlc_cbpy_standard[0], VLC_CBPY_STANDARD_NUM );
    huffman_sort_codes( vlc_cbpy_standard_tree );

    vlc_tcoeff_tree = huffman_alloc(VLC_TCOEFF_NUM, VLC_TCOEFF_MAX_LENGTH);
    huffman_add_codes( vlc_tcoeff_tree, &vlc_tcoeff[0], VLC_TCOEFF_NUM );
    huffman_sort_codes( vlc_tcoeff_tree );

    mvd_vlc_tree = huffman_alloc(MVD_VLC_NUM, MVD_VLC_MAX_LENGTH);
    huffman_add_codes( mvd_vlc_tree, &mvd_vlc[0], MVD_VLC_NUM );
    huffman_sort_codes( mvd_vlc_tree );

    first_init = 0;
  }

  video_codec = (video_codec_t*) vp_os_malloc( sizeof(p263_codec) );

  vp_os_memcpy(video_codec, &p263_codec, sizeof(p263_codec));

  controller->video_codec     = video_codec;
}

void p263_codec_free( video_controller_t* controller )
{
  p263_codec_t* p263_codec = (p263_codec_t*) controller->video_codec;

  vp_os_free( p263_codec );
}

static INLINE video_macroblock_t* p263_unquantize_idct( video_controller_t* controller, video_macroblock_t* mb, int32_t num_macro_blocks )
{
  video_unquantize( controller, mb, num_macro_blocks );
  video_idct_compute(mb->data, mb->data, num_macro_blocks);

  return mb+num_macro_blocks;
}

C_RESULT p263_packet( video_controller_t* controller )
{
  uint32_t start_code = 0, num_mb;
  bool_t picture_complete;
  video_stream_t* stream = &controller->in_stream;
  video_macroblock_t* mb;
  p263_gob_layer_t* gob;
  p263_codec_t* p263_codec = (p263_codec_t*) controller->video_codec;
  p263_picture_layer_t* picture_layer = &p263_codec->picture_layer;

  switch( controller->mode )
  {
  case VIDEO_ENCODE:
    break;

  case VIDEO_DECODE:
    // TODO Fill first gob with picture header's data
    video_align8( stream );
    video_read_data( stream, &start_code, 22 );

    controller->blockline = start_code & 0x1F;
    start_code &= ~0x1F; // TODO Check if compiler use arm instruction bic

    if( start_code == PICTURE_START_CODE )
    {
      picture_complete = TRUE;
      if( controller->blockline == 0 )
      {
        picture_complete = FALSE;
        p263_read_picture_layer( controller, stream );

        picture_layer->gobs = (p263_gob_layer_t*) controller->gobs;
        gob = &picture_layer->gobs[controller->blockline];

        gob->gquant = picture_layer->pquant;
      }
      else if( controller->blockline < 0x1F )
      {
        picture_complete = FALSE;
        p263_read_gob_layer( controller, stream );
      }

      controller->picture_complete = picture_complete;

      if( picture_complete != TRUE )
      {
        num_mb = controller->mb_blockline;
        mb = picture_layer->gobs[controller->blockline].macroblocks;

        for( ; num_mb > 0; num_mb-- )
        {
          p263_read_mb_layer( controller, stream, mb++ );
        }
      }
    }
    break;

  default:
    break;
  }

  return C_OK;
}

C_RESULT p263_encode_blockline( video_controller_t* controller, const vp_api_picture_t* blockline, bool_t picture_complete )
{
  return C_OK;
}

C_RESULT p263_decode_blockline( video_controller_t* controller, vp_api_picture_t* picture, bool_t* got_image )
{
  video_codec_t* video_codec;
  vp_api_picture_t blockline = { 0 };
  int16_t *in = NULL;
  int32_t num_macro_blocks = 0;
  video_macroblock_t* macroblock = NULL;
  video_picture_context_t blockline_ctx;
  video_gob_t*  gobs;

  controller->mode  = VIDEO_DECODE;
  video_codec       = controller->video_codec;

  blockline                   = *picture;
  blockline.height            = MB_HEIGHT_Y;
  blockline.complete          = 1;
  blockline.vision_complete   = 0;

  picture->complete  = controller->picture_complete;

  blockline_ctx.y_woffset = blockline.y_line_size;
  blockline_ctx.c_woffset = blockline.cb_line_size;
  blockline_ctx.y_hoffset = blockline.y_line_size * MCU_HEIGHT;

  // At least a complete blockline is found
  while( !controller->picture_complete && controller->in_stream.index < (controller->in_stream.used >> 2) )
  {
    p263_packet( controller );

    if( !controller->picture_complete )
    {
      blockline.blockline  = controller->blockline;

      blockline_ctx.y_src     = picture->y_buf + blockline.blockline * MB_HEIGHT_Y * picture->y_line_size;
      blockline_ctx.cb_src    = picture->cb_buf + blockline.blockline * MB_HEIGHT_C * picture->cb_line_size;
      blockline_ctx.cr_src    = picture->cr_buf + blockline.blockline * MB_HEIGHT_C * picture->cr_line_size;

      picture->blockline  = controller->blockline;
      num_macro_blocks    = controller->mb_blockline;

      gobs        = &controller->gobs[controller->blockline];
      macroblock  = &controller->gobs[controller->blockline].macroblocks[0];
      in          = macroblock->data;

      if( gobs->quant != controller->quant )
      {
        controller->quant = gobs->quant;
        video_quantizer_update( controller );
      }

      while( num_macro_blocks > MAX_NUM_MACRO_BLOCKS_PER_CALL )
      {
        // These two calls are merged to allow specific optimization in case of a software decoder
        macroblock = p263_unquantize_idct( controller, macroblock, MAX_NUM_MACRO_BLOCKS_PER_CALL );

        num_macro_blocks -= MAX_NUM_MACRO_BLOCKS_PER_CALL;
      }

      // These two calls are merged to allow specific optimization in case of a software decoder
      p263_unquantize_idct( controller, macroblock, num_macro_blocks );

      video_blockline_from_macro_blocks(&blockline_ctx, in, controller->mb_blockline, picture->format);

      // Update controller according to video statistics
      video_controller_update( controller, controller->picture_complete );

      // Perform motion compensation
      if( controller->use_me == TRUE )
      {
      }
    }
  }

  if( controller->picture_complete )
  {
    picture->complete   = controller->picture_complete;
    picture->blockline  = 0;

    controller->picture_complete  = 0;
    controller->in_stream.length  = 32;
    //controller->num_frames++;

    *got_image = TRUE;
  }
  else
  {
    controller->in_stream.used  = 0;
    controller->in_stream.index = 0;
  }

  return C_OK;
}

C_RESULT p263_update( video_controller_t* controller )
{
  return C_OK;
}

static C_RESULT p263_flush_stream( video_stream_t* out, video_stream_t* in )
{
  // They are still data in cache
  // Always copy a number of bytes that is a times of 4.
  // Only for the last copy, we can have exactly the number of bytes left
  int32_t offset, size;
  uint32_t out_stream_size;

  if( in->length != 32 )
  {
    // flush & reset internal stream
    video_write_data( in, 0, in->length+1 );
    in->length = 32;
  }

  out_stream_size = out->size & ~3; // Round to the highest times of 4 available

  offset = in->index - (in->used >> 2);
  size = ( in->used < out_stream_size ) ? in->used : out_stream_size;

  vp_os_memcpy(out->bytes, in->bytes + offset, size);

  out->index  = size >> 2;
  out->used   = size;

  in->used -= size;

  return C_OK;
}

static C_RESULT p263_load_stream( video_stream_t* out, video_stream_t* in )
{
  // We cache as many blockline as possible
  C_RESULT res;
  bool_t found, last_zero, last_zero_temp;
  uint32_t *dst, *src;

  int32_t value, nb_dwords;
  uint32_t in_index = (in->used >> 2) - 1;

  // -> start looking for last blockline's end
  found     = FALSE;

  if( in->index == 0 ) // First call, we look for full blocklines
  {
    last_zero = FALSE;

    while( (in_index > in->index) && !found )
    {
      value = in->bytes[in_index];

      last_zero_temp = (value & 0xFF) == 0; // 0x??????00
      found = last_zero_temp & last_zero;

      if( !found )
      {
        last_zero = last_zero_temp;
        value >>= 8;

        last_zero_temp = (value & 0xFF) == 0; // 0x????00??
        found = last_zero_temp & last_zero;

        if( !found )
        {
          last_zero = last_zero_temp;
          value >>= 8;

          last_zero_temp = (value & 0xFF) == 0; // 0x??00????
          found = last_zero_temp & last_zero;

          if( !found )
          {
            in_index--; // Handle both the special case where blockline is dword aligned &
                        // blockline start is still not found

            last_zero = last_zero_temp;
            value >>= 8;

            last_zero_temp = (value & 0xFF) == 0; // 0x00??????
            found = last_zero_temp & last_zero;

            if( !found )
            {
              last_zero = last_zero_temp;
            }
          }
        }
      }
    }
  }

  in_index++;

  dst   = &out->bytes[0];
  dst  += out->used/4;

  src   = &in->bytes[0];
  src  += in->index;

  // configure parameters for memcpy
  if( !found )
  {
    // cache all data
    // if out->used is non zero then we have already a partial blockline in cache
    nb_dwords = (in->used >> 2) - in->index;

    res = C_FAIL;
  }
  else
  {
    // cache only data containing full blocklines
    // if out->used is non zero then we have already a partial blockline in cache
    nb_dwords = in_index - in->index;

    res = C_OK;
  }

  if( out->used + nb_dwords*4 >= out->size )
  { // Saturates value
    nb_dwords = out->size*4 - out->used;
  }

  if( out->endianess == in->endianess )
  {
    video_copy32( dst, src, nb_dwords );
  }
  else
  {
    // swap copy
    video_copy32_swap( dst, src, nb_dwords );
  }

  out->used += nb_dwords*4;
  in->index  = in_index;

  VP_OS_ASSERT( out->used <= out->size );

  return res;
}

C_RESULT p263_cache( video_controller_t* controller, video_stream_t* ex_stream)
{
  C_RESULT res;

  video_stream_t* in_stream = &controller->in_stream;

  switch( controller->mode )
  {
  case VIDEO_ENCODE:
    res = p263_flush_stream( ex_stream, in_stream );
    break;

  case VIDEO_DECODE:
    res = p263_load_stream( in_stream, ex_stream );
    break;

  default:
    res = C_FAIL;
    break;
  }

  return res;
}
