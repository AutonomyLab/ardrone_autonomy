#include <VLIB/Platform/video_utils.h>
#include <VLIB/Platform/video_config.h>

#include "video_p264.h"
#include <VLIB/video_packetizer.h>
#include "p264_codec.h"
#include <VLIB/video_quantizer.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_types.h>
#include <VP_Os/elinux/vp_os_ltt.h>

#include <VP_Os/vp_os_print.h>

#define NB_P_FRAMES     2
#define P264_DEFAULT_QUANTIZATION 20

const p264_codec_t p264_codec = {
  p264_encode_blockline,
  p264_decode_blockline,
  p264_update,
  p264_cache,
  { 0 }
};

void p264_codec_alloc( video_controller_t* controller )
{
  video_codec_t* video_codec;

  video_codec = (video_codec_t*) vp_os_malloc( sizeof(p264_codec) );

  vp_os_memcpy(video_codec, &p264_codec, sizeof(p264_codec));

  controller->video_codec = video_codec;

}

void p264_realloc_ref (video_controller_t* controller)
{
  // realloc internal p264 buffers and make last decoded picture as the reference
  p264_codec_t* video_codec;
  video_codec = (p264_codec_t*)controller->video_codec;
  if (controller->width != video_codec->ref_picture.width && controller->height != video_codec->ref_picture.height)
  {
    // resolution has changed, realloc buffers
    video_codec->ref_picture.width  = controller->width;
    video_codec->ref_picture.height = controller->height;
    video_codec->decoded_picture.width  = controller->width;
    video_codec->decoded_picture.height = controller->height;

    // allocate a YUV 4:2:0 ref frame
    video_codec->ref_picture.y_buf = (uint8_t*)vp_os_realloc(video_codec->ref_picture.y_buf,controller->width*controller->height*3/2);
    video_codec->decoded_picture.y_buf = (uint8_t*)vp_os_realloc(video_codec->decoded_picture.y_buf,controller->width*controller->height*3/2);
    if (video_codec->ref_picture.y_buf == NULL || video_codec->decoded_picture.y_buf == NULL)
    {
      PRINT("p264 ref realloc failed\n");
    }

    // fill cb/cr fields
    video_codec->ref_picture.cb_buf = video_codec->ref_picture.y_buf + video_codec->ref_picture.width*video_codec->ref_picture.height;
    video_codec->ref_picture.cr_buf = video_codec->ref_picture.cb_buf + (video_codec->ref_picture.width*video_codec->ref_picture.height)/4;
    video_codec->ref_picture.y_line_size = video_codec->ref_picture.width;
    video_codec->ref_picture.cb_line_size = video_codec->ref_picture.width>>1;
    video_codec->ref_picture.cr_line_size = video_codec->ref_picture.width>>1;

    video_codec->decoded_picture.cb_buf = video_codec->decoded_picture.y_buf + video_codec->decoded_picture.width*video_codec->decoded_picture.height;
    video_codec->decoded_picture.cr_buf = video_codec->decoded_picture.cb_buf + (video_codec->decoded_picture.width*video_codec->decoded_picture.height)/4;
    video_codec->decoded_picture.y_line_size = video_codec->decoded_picture.width;
    video_codec->decoded_picture.cb_line_size = video_codec->decoded_picture.width>>1;
    video_codec->decoded_picture.cr_line_size = video_codec->decoded_picture.width>>1;

  }
  // swap decoded_picture and ref picture
  uint8_t* p_swap;
  // swap y
  p_swap = video_codec->ref_picture.y_buf;
  video_codec->ref_picture.y_buf = video_codec->decoded_picture.y_buf;
  video_codec->decoded_picture.y_buf = p_swap;
  // swap cb
  p_swap = video_codec->ref_picture.cb_buf;
  video_codec->ref_picture.cb_buf = video_codec->decoded_picture.cb_buf;
  video_codec->decoded_picture.cb_buf = p_swap;
  // swap cr
  p_swap = video_codec->ref_picture.cr_buf;
  video_codec->ref_picture.cr_buf = video_codec->decoded_picture.cr_buf;
  video_codec->decoded_picture.cr_buf = p_swap;
}

void p264_codec_free( video_controller_t* controller )
{
  p264_codec_t* p264_codec = (p264_codec_t*) controller->video_codec;

  if( p264_codec != NULL )
  {
    if (p264_codec->ref_picture.y_buf != NULL)
    {
      vp_os_free(p264_codec->ref_picture.y_buf);
      p264_codec->ref_picture.y_buf = NULL;
    }
    if (p264_codec->decoded_picture.y_buf != NULL)
    {
      vp_os_free(p264_codec->decoded_picture.y_buf);
      p264_codec->decoded_picture.y_buf = NULL;
    }

    vp_os_free( p264_codec );
    controller->video_codec = NULL;
  }
}


static C_RESULT p264_flush_stream( video_stream_t* out, video_stream_t* in )
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

static C_RESULT p264_load_stream( video_stream_t* out, video_stream_t* in )
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

C_RESULT p264_pack_controller( video_controller_t* controller )
{
  video_stream_t* stream = &controller->in_stream;
  p264_codec_t* p264_codec = (p264_codec_t*) controller->video_codec;
  p264_picture_layer_t* picture_layer;
  p264_gob_layer_t* gob;

  gob = NULL;
  picture_layer = &p264_codec->picture_layer;

  video_stuff8( stream );

  picture_layer->gobs = (p264_gob_layer_t*) controller->gobs;
  gob = &picture_layer->gobs[controller->blockline];

  video_write_data( stream, MAKE_START_CODE(controller->blockline), 22 );

  if( controller->blockline == 0 )
  {
    picture_layer->quant = gob->quant;
    p264_write_picture_layer( controller, stream );
  }
  else
  {
    p264_write_gob_layer( stream, gob );
  }

  return C_OK;
}

C_RESULT p264_unpack_controller( video_controller_t* controller )
{
  uint32_t start_code = 0;
  video_stream_t* stream = &controller->in_stream;
  p264_codec_t* p264_codec = (p264_codec_t*) controller->video_codec;
  p264_picture_layer_t* picture_layer;
  p264_gob_layer_t* gob;

  gob = NULL;
  picture_layer = &p264_codec->picture_layer;

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
        uint32_t last_frame_decoded = controller->num_frames;
        // new picture
        p264_read_picture_layer( controller, stream );

            if (((controller->num_frames == (last_frame_decoded + 1)) && (controller->last_frame_decoded == TRUE))
                || (controller->picture_type == VIDEO_PICTURE_INTRA))
            {
                // new picture is decodable because it's an I frame or previous frame was decodable
                controller->last_frame_decoded = TRUE;
            }
            else
            {
                controller->last_frame_decoded = FALSE;
        }
          
        p264_realloc_ref(controller);
        picture_layer->gobs = (p264_gob_layer_t*) controller->gobs;
        gob = &picture_layer->gobs[controller->blockline];

        gob->quant = picture_layer->quant;
      }
      else
      {
        picture_layer->gobs = (p264_gob_layer_t*) controller->gobs;
        gob = &picture_layer->gobs[controller->blockline];

        p264_read_gob_layer( stream, gob );
      }
    }
  }

  return C_OK;
}

C_RESULT p264_encode_blockline( video_controller_t* controller, const vp_api_picture_t* blockline, bool_t picture_complete )
{
  video_codec_t* video_codec;
  //int16_t *in = NULL;//, *out = NULL;
  int32_t num_macro_blocks = 0;
  video_macroblock_t* macroblock = NULL;
  //video_picture_context_t blockline_ctx;
  video_gob_t*  gobs;

  video_stream_t* stream = &controller->in_stream;

  if( stream->used*2 >= stream->size )
  {
    uint32_t add = 32 - clz(stream->used/controller->blockline);      // estimate the log2 size of a blockline in the stream
    add = 1<<(add+1);                           // major and compute addition buffer size
    stream->bytes = vp_os_realloc( stream->bytes, stream->size + add ); // Add some byte to internal stream
    stream->size += add;
  }

  video_codec                   = controller->video_codec;
  controller->picture_complete  = picture_complete;
  controller->blockline         = blockline->blockline;

  /*
  blockline_ctx.y_src     = blockline->y_buf;
  blockline_ctx.cb_src    = blockline->cb_buf;
  blockline_ctx.cr_src    = blockline->cr_buf;
  blockline_ctx.y_woffset = blockline->y_line_size;
  blockline_ctx.c_woffset = blockline->cb_line_size;
  blockline_ctx.y_hoffset = blockline->y_line_size * MCU_HEIGHT;
  */


  gobs        = &controller->gobs[controller->blockline];
  gobs->quant = controller->quant;
  macroblock  = &gobs->macroblocks[0];

  if (blockline->blockline == 0)
  {
    if (controller->resolution_changed == TRUE)
    {
      // resolution changed, reset ip_counter to force I frame encoding
      ((p264_codec_t*)video_codec)->ip_counter = 0;
    }
    if ((((p264_codec_t*)video_codec)->ip_counter%(NB_P_FRAMES+1))==0)
    {
      video_controller_set_picture_type( controller, VIDEO_PICTURE_INTRA );
      ((p264_codec_t*)(controller->video_codec))->picture_layer.picture_type = VIDEO_PICTURE_INTRA;
    }
    else
    {
       video_controller_set_picture_type( controller, VIDEO_PICTURE_INTER );
       ((p264_codec_t*)(controller->video_codec))->picture_layer.picture_type = VIDEO_PICTURE_INTER;
    }
    ((p264_codec_t*)video_codec)->ip_counter++;

    // it's a new picture, prepare slice
    video_p264_prepare_slice (controller,blockline);
  }

  p264_pack_controller( controller );

  num_macro_blocks = controller->mb_blockline;
  ///>

  if (blockline->blockline == 0)
    // TODO: next line is possible only when full frame is available (i.e. blokline mode not enabled). should not assume that.
    video_p264_encode_MB(controller->mb_blockline*controller->num_blockline,macroblock, gobs->quant);

  while (num_macro_blocks)
  {
    int32_t num_mb_ready = video_p264_get_encoded_MB(num_macro_blocks,macroblock);
    if (num_mb_ready>0)
    {
      num_macro_blocks -= num_mb_ready;
      p264_write_mb_layer(controller, stream, macroblock, num_mb_ready );
      macroblock += num_mb_ready;
    }
  }
  ///<

  ///> Packetize Data to output buffer
  RTMON_USTART(VIDEO_VLIB_PACKET);
  macroblock  = &gobs->macroblocks[0];

  ///> Control Stream size

  if (controller->target_size > 0)
  {
    // compute i_p_ratio && target_size
    uint32_t target_size;
    float32_t i_p_ratio;
    if (((p264_codec_t*)video_codec)->last_I_size !=0 && ((p264_codec_t*)video_codec)->last_P_size !=0)
      i_p_ratio = (float32_t)((p264_codec_t*)video_codec)->last_P_size/(float32_t)((p264_codec_t*)video_codec)->last_I_size;
    else
      i_p_ratio = 3/5;

    if (controller->picture_type == VIDEO_PICTURE_INTRA)
      target_size = (NB_P_FRAMES + 1)*controller->target_size/(1+NB_P_FRAMES*i_p_ratio);
    else
      target_size = (NB_P_FRAMES + 1)*controller->target_size*i_p_ratio/(1+NB_P_FRAMES*i_p_ratio);

    if ((controller->blockline+1) == controller->num_blockline)
    {
      // end of picture
      if ((target_size > stream->used) && controller->quant > 1)
      {
      // last frame was too small increase overall quality (i.e. decrease quant)
        controller->quant--;
      }
      else if ((target_size < stream->used) && controller->quant < 51)
      {
      // last frame was too large decrease overall quality (i.e. increase quant)
        controller->quant++;
      }

      // update las_x_size infos
      if (controller->picture_type == VIDEO_PICTURE_INTRA)
      {
        ((p264_codec_t*)video_codec)->last_I_size = stream->used;
      }
      else
      {
        ((p264_codec_t*)video_codec)->last_P_size = stream->used;
      }
    }
  }
  else
    controller->quant= P264_DEFAULT_QUANTIZATION;

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

#ifndef HAS_P264_DECODE_BLOCKLINE
C_RESULT p264_decode_blockline( video_controller_t* controller, vp_api_picture_t* picture, bool_t* got_image )
{
  p264_codec_t* video_codec;
  vp_api_picture_t blockline = { 0 };
  int32_t num_macro_blocks = 0;
  video_macroblock_t* macroblock = NULL;
  MB_p264_t* p264_mb = NULL;
  video_picture_context_t blockline_ctx;
  video_picture_context_t blockline_src;
  video_gob_t*  gobs;
  uint32_t x_luma=0,y_luma=0;
  uint32_t x_chroma=0,y_chroma=0;

  controller->mode  = VIDEO_DECODE;
  video_codec       = (p264_codec_t*)controller->video_codec;

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
    p264_unpack_controller( controller );
    // update controller picture type
    controller->picture_type = video_codec->picture_layer.picture_type;

    if( !controller->picture_complete )
    {
      blockline.blockline  = controller->blockline;

      blockline_ctx.y_src     = picture->y_buf + blockline.blockline * MB_HEIGHT_Y * picture->y_line_size;
      blockline_ctx.cb_src    = picture->cb_buf + blockline.blockline * MB_HEIGHT_C * picture->cb_line_size;
      blockline_ctx.cr_src    = picture->cr_buf + blockline.blockline * MB_HEIGHT_C * picture->cr_line_size;

      picture->blockline  = controller->blockline;
      num_macro_blocks    = controller->mb_blockline;

      gobs        = &controller->gobs[controller->blockline];
      // contrairement a UVLC, macroblock pointe dans le cache et non pas dans le cache_mbs
      macroblock  = gobs->macroblocks;

      if( gobs->quant != controller->quant )
      {
        controller->quant = gobs->quant;
        video_quantizer_update( controller );
      }

      // compute luma/chroma block (x,y) destination
      x_luma = 0;
      y_luma = blockline.blockline * MB_HEIGHT_Y;
      x_chroma = 0;
      y_chroma = blockline.blockline * MB_HEIGHT_C;

      while( num_macro_blocks > 0 )
      {
        p264_mb = (MB_p264_t*)macroblock->data;
        // entropic decoding
        p264_read_mb_layer(controller, &controller->in_stream, controller->gobs, controller->blockline, controller->mb_blockline-num_macro_blocks);

        if (controller->picture_type == VIDEO_PICTURE_INTER)
        {
          video_p264_decode_inter_luma_MB (     video_codec->ref_picture.y_buf,
                                                video_codec->decoded_picture.y_buf,
                                                x_luma, y_luma,
                                                video_codec->decoded_picture.width, video_codec->decoded_picture.height,
                                                video_codec->decoded_picture.y_line_size,
                                                macroblock->inter_MV,
                                                macroblock->inter_partition_mode,
                                                macroblock->nb_partition,
                                                p264_mb->inter.AC_Y,
                                                gobs->quant);

          video_p264_decode_inter_chroma_MB (   video_codec->ref_picture.cb_buf,
                                                video_codec->decoded_picture.cb_buf,
                                                x_chroma, y_chroma,
                                                video_codec->decoded_picture.width>>1, video_codec->decoded_picture.height>>1,
                                                video_codec->decoded_picture.cb_line_size,
                                                macroblock->inter_MV,
                                                macroblock->inter_partition_mode,
                                                macroblock->nb_partition,
                                                p264_mb->inter.DC_U,
                                                p264_mb->inter.AC_U,
                                                gobs->quant);

          video_p264_decode_inter_chroma_MB (   video_codec->ref_picture.cr_buf,
                                                video_codec->decoded_picture.cr_buf,
                                                x_chroma, y_chroma,
                                                video_codec->decoded_picture.width>>1, video_codec->decoded_picture.height>>1,
                                                video_codec->decoded_picture.cr_line_size,
                                                macroblock->inter_MV,
                                                macroblock->inter_partition_mode,
                                                macroblock->nb_partition,
                                                p264_mb->inter.DC_V,
                                                p264_mb->inter.AC_V,
                                                gobs->quant);
        }
        else if (macroblock->intra_type == INTRA_4x4)
        {
          video_p264_decode_intra_luma_4x4_MB   (p264_mb->intra_4x4.AC_Y,
                                                 video_codec->decoded_picture.y_buf, video_codec->decoded_picture.width,
                                                 x_luma, y_luma, video_codec->decoded_picture.y_line_size,
                                                 macroblock->intra_4x4_mode,
                                                 gobs->quant);

          video_p264_decode_intra_chroma_8x8_MB (p264_mb->intra_4x4.DC_U,
                                                 p264_mb->intra_4x4.AC_U,
                                                 video_codec->decoded_picture.cb_buf,
                                                 x_chroma, y_chroma, video_codec->decoded_picture.cb_line_size,
                                                 macroblock->intra_chroma_8x8_mode,
                                                 gobs->quant);

          video_p264_decode_intra_chroma_8x8_MB (p264_mb->intra_4x4.DC_V,
                                                 p264_mb->intra_4x4.AC_V,
                                                 video_codec->decoded_picture.cr_buf,
                                                 x_chroma, y_chroma, video_codec->decoded_picture.cr_line_size,
                                                 macroblock->intra_chroma_8x8_mode,
                                                 gobs->quant);
        }
        else
        {
          video_p264_decode_intra_luma_16x16_MB (p264_mb->intra_16x16.DC_Y,
                                                   p264_mb->intra_16x16.AC_Y,
                                                   video_codec->decoded_picture.y_buf,
                                                   x_luma, y_luma,  video_codec->decoded_picture.y_line_size,
                                                   macroblock->intra_luma_16x16_mode,
                                                   gobs->quant);

          video_p264_decode_intra_chroma_8x8_MB (p264_mb->intra_16x16.DC_U,
                                                 p264_mb->intra_16x16.AC_U,
                                                 video_codec->decoded_picture.cb_buf,
                                                 x_chroma, y_chroma, video_codec->decoded_picture.cb_line_size,
                                                 macroblock->intra_chroma_8x8_mode,
                                                 gobs->quant);

          video_p264_decode_intra_chroma_8x8_MB (p264_mb->intra_16x16.DC_V,
                                                 p264_mb->intra_16x16.AC_V,
                                                 video_codec->decoded_picture.cr_buf,
                                                 x_chroma, y_chroma, video_codec->decoded_picture.cr_line_size,
                                                 macroblock->intra_chroma_8x8_mode,
                                                 gobs->quant);
        }

        // compute next block coordinates
        x_luma += MB_HEIGHT_Y;
        x_chroma += MB_HEIGHT_C;
        macroblock++;
        num_macro_blocks --;
      }

      // prepare the source picture context
      blockline_src.y_woffset = video_codec->decoded_picture.y_line_size;
      blockline_src.c_woffset = video_codec->decoded_picture.cb_line_size;
      blockline_src.y_hoffset = video_codec->decoded_picture.y_line_size * MCU_HEIGHT;
      blockline_src.y_src     = video_codec->decoded_picture.y_buf + blockline.blockline * MB_HEIGHT_Y * video_codec->decoded_picture.y_line_size;
      blockline_src.cb_src    = video_codec->decoded_picture.cb_buf + blockline.blockline * MB_HEIGHT_C * video_codec->decoded_picture.cb_line_size;
      blockline_src.cr_src    = video_codec->decoded_picture.cr_buf + blockline.blockline * MB_HEIGHT_C * video_codec->decoded_picture.cr_line_size;
      // convert src to dest
      video_blockline_from_blockline(&blockline_ctx, &blockline_src, controller->mb_blockline, picture->format);

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

C_RESULT p264_update( video_controller_t* controller )
{
  return C_OK;
}

C_RESULT p264_cache( video_controller_t* controller, video_stream_t* ex_stream)
{
  C_RESULT res;

  video_stream_t* in_stream = &controller->in_stream;

  switch( controller->mode )
  {
  case VIDEO_ENCODE:
    res = p264_flush_stream( ex_stream, in_stream );
    break;

  case VIDEO_DECODE:
    res = p264_load_stream( in_stream, ex_stream );
    break;

  default:
    res = C_FAIL;
    break;
  }

  return res;
}
