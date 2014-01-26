#include <VLIB/Platform/video_utils.h>
#include <VLIB/Platform/video_config.h>

#include <VLIB/video_quantizer.h>
#include <VLIB/video_dct.h>
#include <VLIB/video_packetizer.h>
#include "uvlc_codec.h"

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_types.h>

const uvlc_codec_t uvlc_codec = {
  uvlc_encode_blockline,
  uvlc_decode_blockline,
  uvlc_update,
  uvlc_cache,
  { 0 }
};

void uvlc_codec_alloc( video_controller_t* controller )
{
  video_codec_t* video_codec;

  video_codec = (video_codec_t*) vp_os_malloc( sizeof(uvlc_codec) );

  vp_os_memcpy(video_codec, &uvlc_codec, sizeof(uvlc_codec));

  controller->video_codec = video_codec;
}

void uvlc_codec_free( video_controller_t* controller )
{
  uvlc_codec_t* uvlc_codec = (uvlc_codec_t*) controller->video_codec;

  if( uvlc_codec != NULL )
  {
    vp_os_free( uvlc_codec );
    controller->video_codec = NULL;
  }
}

static C_RESULT uvlc_flush_stream( video_stream_t* out, video_stream_t* in )
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

static C_RESULT uvlc_load_stream( video_stream_t* out, video_stream_t* in )
{
  // We cache as many blockline as possible
  C_RESULT res;
  bool_t found, last_zero, last_zero_temp;
  uint8_t *dst, *src;

  int32_t value, nb_bytes;
  uint32_t in_index = (in->used >> 2) - 1;

  // -> start looking for last blockline's end
  found = FALSE;

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

  // configure parameters for memcpy
  if( !found )
  {
    // cache all data
    nb_bytes = in->used - in->index * 4;

    res = C_FAIL;
  }
  else
  {
    // cache only data containing full blocklines
    nb_bytes = (in_index - in->index) * 4;

    res = C_OK;
  }

  // Realloc internal stream to have enough space to hold all required data
  while( out->used + nb_bytes >= out->size )
  {
    out->bytes = vp_os_realloc( out->bytes, out->size + 2048 ); // Add 2KB to internal stream
    out->size += 2048;
  }

  dst   = (uint8_t*)&out->bytes[0];
  dst  += out->used;

  src   = (uint8_t*)&in->bytes[0];
  src  += in->index*4;

  vp_os_memcpy( dst, src, nb_bytes );

  out->used += nb_bytes;
  in->index  = in_index;

  VP_OS_ASSERT( out->used <= out->size );

  return res;
}

C_RESULT uvlc_pack_controller( video_controller_t* controller )
{
  video_stream_t* stream = &controller->in_stream;
  uvlc_codec_t* uvlc_codec = (uvlc_codec_t*) controller->video_codec;
  uvlc_picture_layer_t* picture_layer;
  uvlc_gob_layer_t* gob;

  gob = NULL;
  picture_layer = &uvlc_codec->picture_layer;

  video_stuff8( stream );

  picture_layer->gobs = (uvlc_gob_layer_t*) controller->gobs;
  gob = &picture_layer->gobs[controller->blockline];

  video_write_data( stream, MAKE_START_CODE(controller->blockline), 22 );

  if( controller->blockline == 0 )
  {
    picture_layer->quant = gob->quant;
    uvlc_write_picture_layer( controller, stream );
  }
  else
  {
    uvlc_write_gob_layer( stream, gob );
  }

  return C_OK;
}

C_RESULT uvlc_unpack_controller( video_controller_t* controller )
{
  uint32_t start_code = 0;
  video_stream_t* stream = &controller->in_stream;
  uvlc_codec_t* uvlc_codec = (uvlc_codec_t*) controller->video_codec;
  uvlc_picture_layer_t* picture_layer;
  uvlc_gob_layer_t* gob;

  gob = NULL;
  picture_layer = &uvlc_codec->picture_layer;

  video_align8( stream );
  video_read_data( stream, &start_code, 22 );

  controller->blockline = start_code & 0x1F;
  start_code &= ~0x1F; // TODO Check if compiler use arm instruction bic

  VP_OS_ASSERT( controller->blockline == 0x1F ||
                controller->num_blockline == 0 || // Check if cache is allocated for current picture
                (controller->num_blockline > 0 && controller->blockline < controller->num_blockline) );

  if( start_code == PICTURE_START_CODE )
  {
    if( controller->blockline == 0x1F )
    {
      controller->picture_complete = TRUE;
    }
    else
    {
      if( controller->blockline == 0 )
      {
        // new picture
        uvlc_read_picture_layer( controller, stream );
            controller->last_frame_decoded = TRUE;
            
        picture_layer->gobs = (uvlc_gob_layer_t*) controller->gobs;
        gob = &picture_layer->gobs[controller->blockline];

        gob->quant = picture_layer->quant;
      }
      else
      {
        picture_layer->gobs = (uvlc_gob_layer_t*) controller->gobs;
        gob = &picture_layer->gobs[controller->blockline];

        uvlc_read_gob_layer( stream, gob );
      }
    }
  }

  return C_OK;
}

C_RESULT uvlc_encode_blockline( video_controller_t* controller, const vp_api_picture_t* blockline, bool_t picture_complete )
{
  //video_codec_t* video_codec;
  int16_t *in = NULL, *out = NULL;
  int32_t num_macro_blocks = 0;
  video_macroblock_t* macroblock = NULL;
  video_picture_context_t blockline_ctx;
  video_gob_t*  gobs;
  static uint32_t mean_Q = 2;

  video_stream_t* stream = &controller->in_stream;

  if( stream->used*2 >= stream->size )
  {
	uint32_t add = 32 - clz(stream->used/controller->blockline); 	    // estimate the log2 size of a blockline in the stream
	add = 1<<(add+1);												    // major and compute addition buffer size
    stream->bytes = vp_os_realloc( stream->bytes, stream->size + add ); // Add some byte to internal stream
    stream->size += add;
  }

  //video_codec                   = controller->video_codec;
  controller->picture_complete  = picture_complete;
  controller->blockline         = blockline->blockline;

  blockline_ctx.y_src     = blockline->y_buf;
  blockline_ctx.cb_src    = blockline->cb_buf;
  blockline_ctx.cr_src    = blockline->cr_buf;
  blockline_ctx.y_woffset = blockline->y_line_size;
  blockline_ctx.c_woffset = blockline->cb_line_size;
  blockline_ctx.y_hoffset = blockline->y_line_size * MCU_HEIGHT;

  gobs        = &controller->gobs[controller->blockline];
  gobs->quant = controller->quant;
  macroblock  = &gobs->macroblocks[0];

  uvlc_pack_controller( controller );

  in  = controller->blockline_cache;
  out = macroblock->data;

  num_macro_blocks = controller->mb_blockline;

  ///> Cache blockline in dct format & perform dct
  while( num_macro_blocks > MAX_NUM_MACRO_BLOCKS_PER_CALL )
  {
    RTMON_USTART(VIDEO_VLIB_BLOCKLINE_TO_MB);
    video_blockline_to_macro_blocks(&blockline_ctx, in, MAX_NUM_MACRO_BLOCKS_PER_CALL);
    RTMON_USTOP(VIDEO_VLIB_BLOCKLINE_TO_MB);


#ifdef HAS_FDCT_QUANT_COMPUTE
    out = video_fdct_quant_compute(in, out, MAX_NUM_MACRO_BLOCKS_PER_CALL,gobs->quant);
#else
    out = video_fdct_compute(in, out, MAX_NUM_MACRO_BLOCKS_PER_CALL);
#endif
    if( in == controller->blockline_cache )
      in += DCT_BUFFER_SIZE;
    else
      in -= DCT_BUFFER_SIZE;

    num_macro_blocks -= MAX_NUM_MACRO_BLOCKS_PER_CALL;
  }

  RTMON_USTART(VIDEO_VLIB_BLOCKLINE_TO_MB);
  video_blockline_to_macro_blocks(&blockline_ctx, in, num_macro_blocks);
  RTMON_USTOP(VIDEO_VLIB_BLOCKLINE_TO_MB);

  RTMON_USTOP(VIDEO_VLIB_BLOCKLINE_TO_MB);

#ifdef HAS_FDCT_QUANT_COMPUTE
  video_fdct_quant_compute(in, out, num_macro_blocks,gobs->quant);
#else
  video_fdct_compute(in, out, num_macro_blocks);
#endif
  ///<

  ///> Do quantification on each macroblock
  RTMON_USTART(VIDEO_VLIB_QUANTIZE);
  video_quantize( controller, &controller->gobs[controller->blockline].macroblocks[0], controller->mb_blockline );
  RTMON_USTOP(VIDEO_VLIB_QUANTIZE);
  ///<

  ///> Packetize Data to output buffer
  RTMON_USTART(VIDEO_VLIB_PACKET);
  uvlc_write_mb_layer( stream, macroblock, controller->mb_blockline );

  ///> Control Stream size

  if (controller->target_size > 0)
  {
    if ((controller->blockline+1) == controller->num_blockline)
    {
      if ((controller->target_size > stream->used) && mean_Q > 2)
      {
    	// last frame was too small increase overall quality (i.e. decrease quant)
        mean_Q--;
      }
      else if ((controller->target_size < stream->used) && mean_Q < 31)
      {
    	// last frame was too large decrease overall quality (i.e. increase quant)
        mean_Q++;
      }
	}

    if (stream->used > ((controller->blockline+1)*controller->target_size*1.05/controller->num_blockline))
    {
      // stream too large, reduce quality
      controller->quant = mean_Q + 1;
    }
    else if (stream->used < ((controller->blockline+1)*controller->target_size*0.95/controller->num_blockline))
    {
      // stream too low, increase quality
      controller->quant = mean_Q - 1;
    }
    else
      controller->quant = mean_Q;

    // quant saturation, 31 is reserved for old TABLE_QUANTIZATION mode (backward compatibility)
    // TODO: quant == 1 doesn't work, find why
    if (controller->quant > 30)
      controller->quant=30;
    else if (controller->quant < 2)
  	    controller->quant=2;
  }
  else
	  controller->quant= DEFAULT_QUANTIZATION;
  RTMON_USTOP(VIDEO_VLIB_PACKET);
  ///<

  if( controller->picture_complete )
  {
    video_stuff8( stream );
    video_write_data( stream, PICTURE_END_CODE, 22 );
  }

  // Update controller according to user inputs & video statistics
  video_controller_update( controller, picture_complete );

  return C_OK;
}

#ifndef HAS_UVLC_DECODE_BLOCKLINE
C_RESULT uvlc_decode_blockline( video_controller_t* controller, vp_api_picture_t* picture, bool_t* got_image )
{
  //video_codec_t* video_codec;
  vp_api_picture_t blockline = { 0 };
  int16_t *in = NULL, *out = NULL;
  int32_t num_macro_blocks = 0;
  video_macroblock_t* macroblock = NULL;
  video_picture_context_t blockline_ctx;
  video_gob_t*  gobs;

  controller->mode  = VIDEO_DECODE;
  //video_codec       = controller->video_codec;

  blockline                   = *picture;
  blockline.height            = MB_HEIGHT_Y;
  blockline.complete          = 1;
  blockline.vision_complete   = 0;

  picture->complete  = controller->picture_complete;

  blockline_ctx.y_woffset = blockline.y_line_size;
  blockline_ctx.c_woffset = blockline.cb_line_size;
  blockline_ctx.y_hoffset = blockline.y_line_size * MCU_HEIGHT;

  // At least a complete blockline is found
  while( !controller->picture_complete && controller->in_stream.index <= (controller->in_stream.used >> 2) )
  {
    uvlc_unpack_controller( controller );

    if( !controller->picture_complete )
    {
      blockline.blockline  = controller->blockline;

      blockline_ctx.y_src     = picture->y_buf + blockline.blockline * MB_HEIGHT_Y * picture->y_line_size;
      blockline_ctx.cb_src    = picture->cb_buf + blockline.blockline * MB_HEIGHT_C * picture->cb_line_size;
      blockline_ctx.cr_src    = picture->cr_buf + blockline.blockline * MB_HEIGHT_C * picture->cr_line_size;

      picture->blockline  = controller->blockline;
      num_macro_blocks    = controller->mb_blockline;

      macroblock  = &controller->cache_mbs[0];
      gobs        = &controller->gobs[controller->blockline];
      out         = gobs->macroblocks->data;

      if( gobs->quant != controller->quant )
      {
        controller->quant = gobs->quant;
        video_quantizer_update( controller );
      }

      while( num_macro_blocks > MAX_NUM_MACRO_BLOCKS_PER_CALL )
      {
        in = &macroblock->data[0];

        uvlc_read_mb_layer( &controller->in_stream, macroblock, MAX_NUM_MACRO_BLOCKS_PER_CALL );

        video_unquantize( controller, macroblock, MAX_NUM_MACRO_BLOCKS_PER_CALL );

        out = video_idct_compute( in, out, MAX_NUM_MACRO_BLOCKS_PER_CALL );

        if( macroblock == &controller->cache_mbs[0] )
          macroblock += MAX_NUM_MACRO_BLOCKS_PER_CALL;
        else
          macroblock -= MAX_NUM_MACRO_BLOCKS_PER_CALL;

        num_macro_blocks -= MAX_NUM_MACRO_BLOCKS_PER_CALL;
      }

      in = macroblock->data;

      uvlc_read_mb_layer( &controller->in_stream, macroblock, num_macro_blocks );

      video_unquantize( controller, macroblock, num_macro_blocks );

      video_idct_compute( in, out, num_macro_blocks );

      video_blockline_from_macro_blocks(&blockline_ctx, gobs->macroblocks->data, controller->mb_blockline, picture->format);

      // Update controller according to video statistics
      video_controller_update( controller, controller->picture_complete );
    }
  }

  if( controller->picture_complete )
  {
    picture->complete   = controller->picture_complete;
    picture->blockline  = 0;

    controller->picture_complete  = 0;
    controller->in_stream.length  = 32;
    //controller->num_frames++;

    *got_image = controller->last_frame_decoded;
  }
  else
  {
    controller->in_stream.used  = 0;
    controller->in_stream.index = 0;
  }

  return C_OK;
}
#endif

C_RESULT uvlc_update( video_controller_t* controller )
{
  return C_OK;
}

C_RESULT uvlc_cache( video_controller_t* controller, video_stream_t* ex_stream)
{
  C_RESULT res;

  video_stream_t* in_stream = &controller->in_stream;

  switch( controller->mode )
  {
  case VIDEO_ENCODE:
    res = uvlc_flush_stream( ex_stream, in_stream );
    break;

  case VIDEO_DECODE:
    res = uvlc_load_stream( in_stream, ex_stream );
    break;

  default:
    res = C_FAIL;
    break;
  }

  return res;
}
