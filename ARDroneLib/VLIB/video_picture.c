#include <VP_Os/vp_os_print.h>

#include <VLIB/Platform/video_utils.h>
#include <VLIB/video_picture.h>
#include <VP_Os/vp_os_malloc.h>

#ifndef HAS_VIDEO_BLOCKLINE_TO_MACRO_BLOCKS

// Convert a 8x8 block of 8 bits data to a 8x8 block of 16 bits data
static void copy_block_8_16(int16_t* dst, int32_t dst_offset, uint8_t* src, int32_t src_offset)
{
  uint32_t* src32 = (uint32_t*) src;
  uint32_t* dst32 = (uint32_t*) dst;

  uint32_t  src_offset32 = src_offset >> 2;
  uint32_t  dst_offset32 = dst_offset >> 1;

  uint32_t  temp;

  int32_t i;

  for( i = 0; i < MCU_BLOCK_SIZE; i += MCU_WIDTH, src32 += src_offset32, dst32 += dst_offset32 )
  {
    temp = *src32++;

    *dst32++ = ((temp << 8) & 0x00FF0000) | (temp & 0x000000FF);
    *dst32++ = ((temp >> 8) & 0x00FF0000) | ((temp >> 16) & 0x000000FF);

    temp = *src32++;

    *dst32++ = ((temp << 8) & 0x00FF0000) | (temp & 0x000000FF);
    *dst32++ = ((temp >> 8) & 0x00FF0000) | ((temp >> 16) & 0x000000FF);
  }

}

#endif

// Convert a 8x8 block of 16 bits data to a 8x8 block of 8 bits data
static void copy_block_16_8(uint8_t* dst, int32_t dst_offset, int16_t* src, int32_t src_offset)
{
  int32_t i;
  int16_t temp;

  for( i = 0; i < MCU_BLOCK_SIZE; i += MCU_WIDTH, dst += dst_offset, src += src_offset )
  {
    temp = *src++; if( temp > 0xff ) temp = 0xff; if(temp < 0) temp = 0; temp &= 0xff; *dst++ = (uint8_t) temp;
    temp = *src++; if( temp > 0xff ) temp = 0xff; if(temp < 0) temp = 0; temp &= 0xff; *dst++ = (uint8_t) temp;
    temp = *src++; if( temp > 0xff ) temp = 0xff; if(temp < 0) temp = 0; temp &= 0xff; *dst++ = (uint8_t) temp;
    temp = *src++; if( temp > 0xff ) temp = 0xff; if(temp < 0) temp = 0; temp &= 0xff; *dst++ = (uint8_t) temp;
    temp = *src++; if( temp > 0xff ) temp = 0xff; if(temp < 0) temp = 0; temp &= 0xff; *dst++ = (uint8_t) temp;
    temp = *src++; if( temp > 0xff ) temp = 0xff; if(temp < 0) temp = 0; temp &= 0xff; *dst++ = (uint8_t) temp;
    temp = *src++; if( temp > 0xff ) temp = 0xff; if(temp < 0) temp = 0; temp &= 0xff; *dst++ = (uint8_t) temp;
    temp = *src++; if( temp > 0xff ) temp = 0xff; if(temp < 0) temp = 0; temp &= 0xff; *dst++ = (uint8_t) temp;
  }
}

//
// Transform blockline in macro blocks
//
// Blockline:
//  _______
// | 1 | 2 |
// |___|___|  Y
// | 3 | 4 |
// |___|___|
//  ___
// | 5 |
// |___| Cb
//  ___
// | 6 |
// |___| Cr
//
// Dct cache:
//  _______________________
// | 1 | 2 | 3 | 4 | 5 | 6 | ...
// |___|___|___|___|___|___|
//

#ifndef HAS_VIDEO_BLOCKLINE_TO_MACRO_BLOCKS

C_RESULT video_blockline_to_macro_blocks(video_picture_context_t* ctx, int16_t* dst, int32_t num_macro_blocks)
{
  uint8_t* y_src        = ctx->y_src;
  uint8_t* cb_src       = ctx->cb_src;
  uint8_t* cr_src       = ctx->cr_src;

  while( num_macro_blocks > 0 )
  {
    // Current MB
    copy_block_8_16( dst,
                     0,
                     y_src,
                     ctx->y_woffset - MCU_WIDTH );

    dst += MCU_BLOCK_SIZE; // skip block 1

    copy_block_8_16( dst,
                     0,
                     y_src + MCU_WIDTH,
                     ctx->y_woffset - MCU_WIDTH );

    dst += MCU_BLOCK_SIZE; // skip block 2

    copy_block_8_16( dst,
                     0,
                     y_src + ctx->y_hoffset,
                     ctx->y_woffset - MCU_WIDTH );

    dst += MCU_BLOCK_SIZE; // skip block 3

    copy_block_8_16( dst,
                     0,
                     y_src + ctx->y_hoffset + MCU_WIDTH,
                     ctx->y_woffset - MCU_WIDTH );

    dst += MCU_BLOCK_SIZE; // skip block 4

    copy_block_8_16( dst,
                     0,
                     cb_src,
                     ctx->c_woffset - MCU_WIDTH );

    dst += MCU_BLOCK_SIZE;  // skip blocks 5

    copy_block_8_16( dst,
                     0,
                     cr_src,
                     ctx->c_woffset - MCU_WIDTH );

    dst += MCU_BLOCK_SIZE;  // skip blocks 6

    y_src   += MCU_WIDTH*2;
    cb_src  += MCU_WIDTH;
    cr_src  += MCU_WIDTH;

    num_macro_blocks --;
  }

  ctx->y_src  = y_src;
  ctx->cb_src = cb_src;
  ctx->cr_src = cr_src;

  return C_OK;
}

#endif

// Transform macro blocks in picture of specified format
static C_RESULT video_blockline_from_macro_blocks_yuv420(video_picture_context_t* ctx, int16_t* src, int32_t num_macro_blocks);
static C_RESULT video_blockline_from_macro_blocks_rgb565(video_picture_context_t* ctx, int16_t* src, int32_t num_macro_blocks);
static C_RESULT video_blockline_from_macro_blocks_rgb24(video_picture_context_t* ctx, int16_t* src, int32_t num_macro_blocks);

C_RESULT video_blockline_from_macro_blocks(video_picture_context_t* ctx, int16_t* src, int32_t num_macro_blocks, enum PixelFormat format)
{
  C_RESULT res;

  switch(format)
  {
    case PIX_FMT_YUV420P:
      res = video_blockline_from_macro_blocks_yuv420(ctx, src, num_macro_blocks);
      break;
    case PIX_FMT_RGB565:
      res = video_blockline_from_macro_blocks_rgb565(ctx, src, num_macro_blocks);
      break;
    case PIX_FMT_RGB24:
      res = video_blockline_from_macro_blocks_rgb24(ctx, src, num_macro_blocks);
      break;

    default:
      PRINT("In file %s, in function %s, format %d not supported\n", __FILE__, __FUNCTION__, format);
      res = C_FAIL;
      break;
  }

  return res;
}

C_RESULT video_blockline_from_macro_blocks_yuv420(video_picture_context_t* ctx, int16_t* src, int32_t num_macro_blocks)
{
  uint8_t *y_dst, *cb_dst, *cr_dst;

  y_dst   = ctx->y_src;
  cb_dst  = ctx->cb_src;
  cr_dst  = ctx->cr_src;

  while( num_macro_blocks > 0 )
  {
    // Current MB
    copy_block_16_8( y_dst,
                     ctx->y_woffset - MCU_WIDTH,
                     src,
                     0 );

    src += MCU_BLOCK_SIZE;

    copy_block_16_8( y_dst + MCU_WIDTH,
                     ctx->y_woffset - MCU_WIDTH,
                     src,
                     0 );

    src += MCU_BLOCK_SIZE;

    copy_block_16_8( y_dst + ctx->y_hoffset,
                     ctx->y_woffset - MCU_WIDTH,
                     src,
                     0 );

    src += MCU_BLOCK_SIZE;

    copy_block_16_8( y_dst + ctx->y_hoffset + MCU_WIDTH,
                     ctx->y_woffset - MCU_WIDTH,
                     src,
                     0 );

    src += MCU_BLOCK_SIZE;

    copy_block_16_8( cb_dst,
                     ctx->c_woffset - MCU_WIDTH,
                     src,
                     0 );

    src += MCU_BLOCK_SIZE;

    copy_block_16_8( cr_dst,
                     ctx->c_woffset - MCU_WIDTH,
                     src,
                     0 );

    src += MCU_BLOCK_SIZE;

    y_dst   += MCU_WIDTH*2;
    cb_dst  += MCU_WIDTH;
    cr_dst  += MCU_WIDTH;

    num_macro_blocks--;
  }

  ctx->y_src  = y_dst;
  ctx->cb_src = cb_dst;
  ctx->cr_src = cr_dst;

  return C_OK;
}

#define MAKE_RGBA_565(r, g, b) ( ((r) << 11) | ((g) << 5) | (b) )

#if TARGET_CPU_ARM == 1 && defined(TARGET_OS_IPHONE)
static inline int32_t saturate8(int32_t x)
{
	usat(x, 8, 8);

	return x;
}

static inline uint32_t saturate5(int32_t x)
{
	usat(x, 5, 11);

	return x;
}

static inline uint32_t saturate6(int32_t x)
{
	usat(x, 6, 10);

	return x;
}

#else

// To make sure that you are bounding your inputs in the range of 0 & 255

static inline int32_t saturate8(int32_t x)
{
  if( x < 0 )
  {
    x = 0;
  }

  x >>= 8;

  return x > 0xFF ? 0xFF : x;
}

static inline uint16_t saturate5(int32_t x)
{
  if( x < 0 )
  {
    x = 0;
  }

  x >>= 11;

  return x > 0x1F ? 0x1F : x;
}

static inline uint16_t saturate6(int32_t x)
{
  if( x < 0 )
  {
    x = 0;
  }

  x >>= 10;

  return x > 0x3F ? 0x3F : x;
}
#endif

static C_RESULT video_blockline_from_macro_blocks_rgb565(video_picture_context_t* ctx, int16_t* src, int32_t num_macro_blocks)
{
  uint32_t y_up_read, y_down_read, cr_current, cb_current;
  int32_t u, v, vr, ug, vg, ub, r, g, b;
  int16_t *y_buf1, *y_buf2, *cr_buf, *cb_buf;
  uint16_t *dst_up, *dst_down;

  // Control variables
  int32_t line_size, block_size, y_woffset, y_hoffset;

  y_buf1  = src;
  y_buf2  = y_buf1 + MCU_WIDTH;

  cb_buf  = y_buf1 + MCU_BLOCK_SIZE * 4;
  cr_buf  = cb_buf + MCU_BLOCK_SIZE;

  // Our ptrs are 16 bits
  y_woffset = ctx->y_woffset / 2;
  y_hoffset = ctx->y_hoffset / 2;

  dst_up    = (uint16_t*) ctx->y_src;
  dst_down  = dst_up + y_woffset;

  line_size		= MCU_WIDTH / 2; // We compute two pixels at a time
  block_size	= MCU_HEIGHT / 2; // We compute two lines at a time

  while( num_macro_blocks > 0 )
  {
    // First quarter
    cb_current  = cb_buf[0];
    cr_current  = cr_buf[0];

    u   = cb_current - 128;
    ug  = 88 * u;
    ub  = 454 * u;
    v   = cr_current - 128;
    vg  = 183 * v;
    vr  = 359 * v;

    y_up_read   = y_buf1[0] << 8;
    y_down_read = y_buf2[0] << 8;

    r = saturate5((y_up_read + vr));
    g = saturate6((y_up_read - ug - vg));
    b = saturate5((y_up_read + ub));

    dst_up[0] = MAKE_RGBA_565(r, g, b);

    r = saturate5((y_down_read + vr));
    g = saturate6((y_down_read - ug - vg));
    b = saturate5((y_down_read + ub));

    dst_down[0] = MAKE_RGBA_565(r, g, b);

    y_up_read   = y_buf1[1] << 8;
    y_down_read = y_buf2[1] << 8;

    r = saturate5((y_up_read + vr));
    g = saturate6((y_up_read - ug - vg));
    b = saturate5((y_up_read + ub));

    dst_up[1] = MAKE_RGBA_565(r, g, b);

    r = saturate5((y_down_read + vr));
    g = saturate6((y_down_read - ug - vg));
    b = saturate5((y_down_read + ub));

    dst_down[1] = MAKE_RGBA_565(r, g, b);

    // Second quarter
    cr_current  = cr_buf[MCU_WIDTH / 2];
    cb_current  = cb_buf[MCU_WIDTH / 2];

    u   = cb_current - 128;
    ug  = 88 * u;
    ub  = 454 * u;
    v   = cr_current - 128;
    vg  = 183 * v;
    vr  = 359 * v;

    y_up_read   = y_buf1[MCU_BLOCK_SIZE] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE] << 8;

    r = saturate5((y_up_read + vr));
    g = saturate6((y_up_read - ug - vg));
    b = saturate5((y_up_read + ub));

    dst_up[MCU_WIDTH] = MAKE_RGBA_565(r, g, b);

    r = saturate5((y_down_read + vr));
    g = saturate6((y_down_read - ug - vg));
    b = saturate5((y_down_read + ub));

    dst_down[MCU_WIDTH] = MAKE_RGBA_565(r, g, b);

    y_up_read   = y_buf1[MCU_BLOCK_SIZE + 1] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE + 1] << 8;

    r = saturate5((y_up_read + vr));
    g = saturate6((y_up_read - ug - vg));
    b = saturate5((y_up_read + ub));

    dst_up[MCU_WIDTH+1] = MAKE_RGBA_565(r, g, b);

    r = saturate5((y_down_read + vr));
    g = saturate6((y_down_read - ug - vg));
    b = saturate5((y_down_read + ub));

    dst_down[MCU_WIDTH+1] = MAKE_RGBA_565(r, g, b);

    // Third quarter
    cr_current  = cr_buf[MCU_BLOCK_SIZE/2];
    cb_current  = cb_buf[MCU_BLOCK_SIZE/2];

    u   = cb_current - 128;
    ug  = 88 * u;
    ub  = 454 * u;
    v   = cr_current - 128;
    vg  = 183 * v;
    vr  = 359 * v;

    y_up_read   = y_buf1[MCU_BLOCK_SIZE*2] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE*2] << 8;

    r = saturate5((y_up_read + vr));
    g = saturate6((y_up_read - ug - vg));
    b = saturate5((y_up_read + ub));

    dst_up[y_hoffset] = MAKE_RGBA_565(r, g, b);

    r = saturate5((y_down_read + vr));
    g = saturate6((y_down_read - ug - vg));
    b = saturate5((y_down_read + ub));

    dst_down[y_hoffset] = MAKE_RGBA_565(r, g, b);

    y_up_read   = y_buf1[MCU_BLOCK_SIZE*2 + 1] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE*2 + 1] << 8;

    r = saturate5((y_up_read + vr));
    g = saturate6((y_up_read - ug - vg));
    b = saturate5((y_up_read + ub));

    dst_up[y_hoffset + 1] = MAKE_RGBA_565(r, g, b);

    r = saturate5((y_down_read + vr));
    g = saturate6((y_down_read - ug - vg));
    b = saturate5((y_down_read + ub));

    dst_down[y_hoffset + 1] = MAKE_RGBA_565(r, g, b);

    // Fourth quarter
    cr_current  = cr_buf[MCU_BLOCK_SIZE/2 + MCU_WIDTH/2];
    cb_current  = cb_buf[MCU_BLOCK_SIZE/2 + MCU_WIDTH/2];

    u   = cb_current - 128;
    ug  = 88 * u;
    ub  = 454 * u;
    v   = cr_current - 128;
    vg  = 183 * v;
    vr  = 359 * v;

    y_up_read   = y_buf1[MCU_BLOCK_SIZE*3] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE*3] << 8;

    r = saturate5((y_up_read + vr));
    g = saturate6((y_up_read - ug - vg));
    b = saturate5((y_up_read + ub));

    dst_up[y_hoffset + MCU_WIDTH] = MAKE_RGBA_565(r, g, b);

    r = saturate5((y_down_read + vr));
    g = saturate6((y_down_read - ug - vg));
    b = saturate5((y_down_read + ub));

    dst_down[y_hoffset + MCU_WIDTH] = MAKE_RGBA_565(r, g, b);

    y_up_read   = y_buf1[MCU_BLOCK_SIZE*3 + 1] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE*3 + 1] << 8;

    r = saturate5((y_up_read + vr));
    g = saturate6((y_up_read - ug - vg));
    b = saturate5((y_up_read + ub));

    dst_up[y_hoffset + MCU_WIDTH + 1] = MAKE_RGBA_565(r, g, b);

    r = saturate5((y_down_read + vr));
    g = saturate6((y_down_read - ug - vg));
    b = saturate5((y_down_read + ub));

    dst_down[y_hoffset + MCU_WIDTH + 1] = MAKE_RGBA_565(r, g, b);

    cr_buf    += 1;
    cb_buf    += 1;
    y_buf1    += 2;
    y_buf2    += 2;
    dst_up    += 2;
    dst_down  += 2;

    line_size--;

    if( line_size == 0 ) // We computed one line of a luma-block
    {
      dst_up    += y_woffset*2 - MCU_WIDTH;
      dst_down  += y_woffset*2 - MCU_WIDTH;

      block_size--;

      if( block_size == 0 )
      {
        y_buf1  = cr_buf + MCU_BLOCK_SIZE/2 + MCU_WIDTH/2;
        y_buf2  = y_buf1 + MCU_WIDTH;

        cb_buf  = y_buf1 + MCU_BLOCK_SIZE * 4;
        cr_buf  = cb_buf + MCU_BLOCK_SIZE;

        block_size = MCU_WIDTH / 2; // We compute two lines at a time

        dst_up  += 2*MCU_WIDTH - y_hoffset;
        dst_down = dst_up + y_woffset;

        num_macro_blocks--;
      }
      else
      {
        y_buf1  = y_buf2;
        y_buf2 += MCU_WIDTH;

        cr_buf += MCU_WIDTH / 2;
        cb_buf += MCU_WIDTH / 2;
      }

      line_size	= MCU_WIDTH / 2; // We compute two pixels at a time
    }
  }

  ctx->y_src = (uint8_t*) dst_up;

  return C_OK;
}

static C_RESULT video_blockline_from_macro_blocks_rgb24(video_picture_context_t* ctx, int16_t* src, int32_t num_macro_blocks)
{
  uint32_t y_up_read, y_down_read, cr_current, cb_current;
  int32_t u, v, vr, ug, vg, ub, r, g, b;
  int16_t *y_buf1, *y_buf2, *cr_buf, *cb_buf;
  uint8_t *dst_up, *dst_down;

  // Control variables
  int32_t line_size, block_size, y_woffset, y_hoffset;

  y_buf1  = src;
  y_buf2  = y_buf1 + MCU_WIDTH;

  cb_buf  = y_buf1 + MCU_BLOCK_SIZE * 4;
  cr_buf  = cb_buf + MCU_BLOCK_SIZE;

  y_woffset = ctx->y_woffset;
  y_hoffset = ctx->y_hoffset;

  dst_up    = (uint8_t*) ctx->y_src;
  dst_down  = dst_up + y_woffset;

  line_size	= MCU_WIDTH / 2; // We compute two pixels at a time
  block_size	= MCU_HEIGHT / 2; // We compute two lines at a time

  while( num_macro_blocks > 0 )
  {
    // First quarter
    cb_current  = cb_buf[0];
    cr_current  = cr_buf[0];

    u   = cb_current - 128;
    ug  = 88 * u;
    ub  = 454 * u;
    v   = cr_current - 128;
    vg  = 183 * v;
    vr  = 359 * v;

    y_up_read   = y_buf1[0] << 8;
    y_down_read = y_buf2[0] << 8;

    r = saturate8((y_up_read + vr));
    g = saturate8((y_up_read - ug - vg));
    b = saturate8((y_up_read + ub));

    dst_up[0] = r;
    dst_up[1] = g;
    dst_up[2] = b;

    r = saturate8((y_down_read + vr));
    g = saturate8((y_down_read - ug - vg));
    b = saturate8((y_down_read + ub));

    dst_down[0] = r;
    dst_down[1] = g;
    dst_down[2] = b;

    y_up_read   = y_buf1[1] << 8;
    y_down_read = y_buf2[1] << 8;

    r = saturate8((y_up_read + vr));
    g = saturate8((y_up_read - ug - vg));
    b = saturate8((y_up_read + ub));

    dst_up[3] = r;
    dst_up[4] = g;
    dst_up[5] = b;

    r = saturate8((y_down_read + vr));
    g = saturate8((y_down_read - ug - vg));
    b = saturate8((y_down_read + ub));

    dst_down[3] = r;
    dst_down[4] = g;
    dst_down[5] = b;

    // Second quarter
    cr_current  = cr_buf[MCU_WIDTH / 2];
    cb_current  = cb_buf[MCU_WIDTH / 2];

    u   = cb_current - 128;
    ug  = 88 * u;
    ub  = 454 * u;
    v   = cr_current - 128;
    vg  = 183 * v;
    vr  = 359 * v;

    y_up_read   = y_buf1[MCU_BLOCK_SIZE] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE] << 8;

    r = saturate8((y_up_read + vr));
    g = saturate8((y_up_read - ug - vg));
    b = saturate8((y_up_read + ub));

    dst_up[3 * MCU_WIDTH + 0] = r;
    dst_up[3 * MCU_WIDTH + 1] = g;
    dst_up[3 * MCU_WIDTH + 2] = b;

    r = saturate8((y_down_read + vr));
    g = saturate8((y_down_read - ug - vg));
    b = saturate8((y_down_read + ub));

    dst_down[3 * MCU_WIDTH + 0] = r;
    dst_down[3 * MCU_WIDTH + 1] = g;
    dst_down[3 * MCU_WIDTH + 2] = b;

    y_up_read   = y_buf1[MCU_BLOCK_SIZE + 1] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE + 1] << 8;

    r = saturate8((y_up_read + vr));
    g = saturate8((y_up_read - ug - vg));
    b = saturate8((y_up_read + ub));

    dst_up[3 * MCU_WIDTH + 3] = r;
    dst_up[3 * MCU_WIDTH + 4] = g;
    dst_up[3 * MCU_WIDTH + 5] = b;

    r = saturate8((y_down_read + vr));
    g = saturate8((y_down_read - ug - vg));
    b = saturate8((y_down_read + ub));

    dst_down[3 * MCU_WIDTH + 3] = r;
    dst_down[3 * MCU_WIDTH + 4] = g;
    dst_down[3 * MCU_WIDTH + 5] = b;

    // Third quarter
    cr_current  = cr_buf[MCU_BLOCK_SIZE/2];
    cb_current  = cb_buf[MCU_BLOCK_SIZE/2];

    u   = cb_current - 128;
    ug  = 88 * u;
    ub  = 454 * u;
    v   = cr_current - 128;
    vg  = 183 * v;
    vr  = 359 * v;

    y_up_read   = y_buf1[MCU_BLOCK_SIZE*2] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE*2] << 8;

    r = saturate8((y_up_read + vr));
    g = saturate8((y_up_read - ug - vg));
    b = saturate8((y_up_read + ub));

    dst_up[y_hoffset + 0] = r;
    dst_up[y_hoffset + 1] = g;
    dst_up[y_hoffset + 2] = b;

    r = saturate8((y_down_read + vr));
    g = saturate8((y_down_read - ug - vg));
    b = saturate8((y_down_read + ub));

    dst_down[y_hoffset + 0] = r;
    dst_down[y_hoffset + 1] = g;
    dst_down[y_hoffset + 2] = b;

    y_up_read   = y_buf1[MCU_BLOCK_SIZE*2 + 1] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE*2 + 1] << 8;

    r = saturate8((y_up_read + vr));
    g = saturate8((y_up_read - ug - vg));
    b = saturate8((y_up_read + ub));

    dst_up[y_hoffset + 3] = r;
    dst_up[y_hoffset + 4] = g;
    dst_up[y_hoffset + 5] = b;

    r = saturate8((y_down_read + vr));
    g = saturate8((y_down_read - ug - vg));
    b = saturate8((y_down_read + ub));

    dst_down[y_hoffset + 3] = r;
    dst_down[y_hoffset + 4] = g;
    dst_down[y_hoffset + 5] = b;

    // Fourth quarter
    cr_current  = cr_buf[MCU_BLOCK_SIZE/2 + MCU_WIDTH/2];
    cb_current  = cb_buf[MCU_BLOCK_SIZE/2 + MCU_WIDTH/2];

    u   = cb_current - 128;
    ug  = 88 * u;
    ub  = 454 * u;
    v   = cr_current - 128;
    vg  = 183 * v;
    vr  = 359 * v;

    y_up_read   = y_buf1[MCU_BLOCK_SIZE*3] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE*3] << 8;

    r = saturate8((y_up_read + vr));
    g = saturate8((y_up_read - ug - vg));
    b = saturate8((y_up_read + ub));

    dst_up[y_hoffset + 3 * MCU_WIDTH + 0] = r;
    dst_up[y_hoffset + 3 * MCU_WIDTH + 1] = g;
    dst_up[y_hoffset + 3 * MCU_WIDTH + 2] = b;

    r = saturate8((y_down_read + vr));
    g = saturate8((y_down_read - ug - vg));
    b = saturate8((y_down_read + ub));

    dst_down[y_hoffset + 3 * MCU_WIDTH + 0] = r;
    dst_down[y_hoffset + 3 * MCU_WIDTH + 1] = g;
    dst_down[y_hoffset + 3 * MCU_WIDTH + 2] = b;

    y_up_read   = y_buf1[MCU_BLOCK_SIZE*3 + 1] << 8;
    y_down_read = y_buf2[MCU_BLOCK_SIZE*3 + 1] << 8;

    r = saturate8((y_up_read + vr));
    g = saturate8((y_up_read - ug - vg));
    b = saturate8((y_up_read + ub));

    dst_up[y_hoffset + 3 * MCU_WIDTH + 3] = r;
    dst_up[y_hoffset + 3 * MCU_WIDTH + 4] = g;
    dst_up[y_hoffset + 3 * MCU_WIDTH + 5] = b;

    r = saturate8((y_down_read + vr));
    g = saturate8((y_down_read - ug - vg));
    b = saturate8((y_down_read + ub));

    dst_down[y_hoffset + 3 * MCU_WIDTH + 3] = r;
    dst_down[y_hoffset + 3 * MCU_WIDTH + 4] = g;
    dst_down[y_hoffset + 3 * MCU_WIDTH + 5] = b;

    cr_buf    += 1;
    cb_buf    += 1;
    y_buf1    += 2;
    y_buf2    += 2;
    dst_up    += 6;
    dst_down  += 6;

    line_size--;

    if( line_size == 0 ) // We computed one line of a luma-block
    {
      dst_up    += y_woffset*2 - (3 * MCU_WIDTH);
      dst_down  += y_woffset*2 - (3 * MCU_WIDTH);

      block_size--;

      if( block_size == 0 )
      {
        y_buf1  = cr_buf + MCU_BLOCK_SIZE/2 + MCU_WIDTH/2;
        y_buf2  = y_buf1 + MCU_WIDTH;

        cb_buf  = y_buf1 + MCU_BLOCK_SIZE * 4;
        cr_buf  = cb_buf + MCU_BLOCK_SIZE;

        block_size = MCU_WIDTH / 2; // We compute two lines at a time

        dst_up  += 6*MCU_WIDTH - y_hoffset;
        dst_down = dst_up + y_woffset;

        num_macro_blocks--;
      }
      else
      {
        y_buf1  = y_buf2;
        y_buf2 += MCU_WIDTH;

        cr_buf += MCU_WIDTH / 2;
        cb_buf += MCU_WIDTH / 2;
      }

      line_size	= MCU_WIDTH / 2; // We compute two pixels at a time
    }
  }

  ctx->y_src = (uint8_t*) dst_up;

  return C_OK;
}



// Transform a blockline YUV 4:2:0 in picture of specified format
static C_RESULT video_blockline_from_blockline_yuv420(video_picture_context_t* ctx, video_picture_context_t* src, int32_t num_macro_blocks);
static C_RESULT video_blockline_from_blockline_rgb565(video_picture_context_t* ctx, video_picture_context_t* src, int32_t num_macro_blocks);
static C_RESULT video_blockline_from_blockline_rgb24(video_picture_context_t* ctx, video_picture_context_t* src, int32_t num_macro_blocks);

C_RESULT video_blockline_from_blockline(video_picture_context_t* ctx, video_picture_context_t* src, int32_t num_macro_blocks, enum PixelFormat format)
{
  C_RESULT res;

  switch(format)
  {
    case PIX_FMT_YUV420P:
      res = video_blockline_from_blockline_yuv420(ctx, src, num_macro_blocks);
      break;
    case PIX_FMT_RGB565:
      res = video_blockline_from_blockline_rgb565(ctx, src, num_macro_blocks);
      break;
    case PIX_FMT_RGB24:
      res = video_blockline_from_blockline_rgb24(ctx, src, num_macro_blocks);
      break;

    default:
      PRINT("In file %s, in function %s, format %d not supported\n", __FILE__, __FUNCTION__, format);
      res = C_FAIL;
      break;
  }

  return res;
}

static C_RESULT video_blockline_from_blockline_yuv420(video_picture_context_t* blockline_ctx, video_picture_context_t* blockline_src, int32_t num_macro_blocks)
{
  uint32_t line;
  uint8_t* dest;
  uint8_t* src;
  uint32_t copy_line_size;

  // copy luma
  copy_line_size = num_macro_blocks * MB_HEIGHT_Y;
  dest = blockline_ctx->y_src;
  src  = blockline_src->y_src;
  line = MB_HEIGHT_Y;
  while (line--)
  {
    vp_os_memcpy(dest,src,copy_line_size);
    dest += blockline_ctx->y_woffset;
    src  += blockline_src->y_woffset;
  }

  // copy cb
  copy_line_size = num_macro_blocks * MB_HEIGHT_C;
  dest = blockline_ctx->cb_src;
  src  = blockline_src->cb_src;
  line = MB_HEIGHT_C;
  while (line--)
  {
    vp_os_memcpy(dest,src,copy_line_size);
    dest += blockline_ctx->c_woffset;
    src  += blockline_src->c_woffset;
  }

  // copy cr
  copy_line_size = num_macro_blocks * MB_HEIGHT_C;
  dest = blockline_ctx->cr_src;
  src  = blockline_src->cr_src;
  line = MB_HEIGHT_C;
  while (line--)
  {
    vp_os_memcpy(dest,src,copy_line_size);
    dest += blockline_ctx->c_woffset;
    src  += blockline_src->c_woffset;
  }

  return C_OK;
}


static C_RESULT
video_blockline_from_blockline_rgb565(video_picture_context_t* ctx,
									  video_picture_context_t* src, int32_t num_macro_blocks)
{
	uint32_t y_up_read, y_down_read, cr_current, cb_current;
	int32_t u, v, vr, ug, vg, ub, r, g, b;
	uint8_t *y_buf_up, *y_buf_down, *cr_buf, *cb_buf;
	uint16_t *dst_up, *dst_down;
	int32_t line_size,dest_y_woffset;
	
	// Control variables
	uint32_t pixel,line;
	
	// In ptrs
	y_buf_up = src->y_src;
	y_buf_down = y_buf_up + src->y_woffset;
	
	cb_buf = src->cb_src;
	cr_buf = src->cr_src;
	
	// Out ptrs are 16 bits
	dest_y_woffset = ctx->y_woffset / 2;
	
	dst_up = (uint16_t*) ctx->y_src;
	dst_down = dst_up + dest_y_woffset;
	
	// We compute two pixels at a time
	line_size = MB_WIDTH_Y*num_macro_blocks; 
	
	pixel = line_size>>1;
	line = MB_WIDTH_Y>>1;
	
	while (line)
	{
		// load cb cr
		cb_current = *cb_buf++;
		cr_current = *cr_buf++;
		
		u = cb_current - 128;
		ug = 88 * u;
		ub = 454 * u;
		v = cr_current - 128;
		vg = 183 * v;
		vr = 359 * v;
		
		// compute pixel(0,0)
		y_up_read = (*y_buf_up++) << 8;
		r = saturate5((y_up_read + vr));
		g = saturate6((y_up_read - ug - vg));
		b = saturate5((y_up_read + ub));
		
		*dst_up++ = MAKE_RGBA_565(r, g, b);
		
		// compute pixel(1,0)
		y_up_read = (*y_buf_up++) << 8;
		
		r = saturate5((y_up_read + vr));
		g = saturate6((y_up_read - ug - vg));
		b = saturate5((y_up_read + ub));
		
		*dst_up++ = MAKE_RGBA_565(r, g, b);
		
		// compute pixel (0,1)
		y_down_read = (*y_buf_down++) << 8;
		
		r = saturate5((y_down_read + vr));
		g = saturate6((y_down_read - ug - vg));
		b = saturate5((y_down_read + ub));
		
		*dst_down++ = MAKE_RGBA_565(r, g, b);
		
		// compute pixel (1,1)
		y_down_read = (*y_buf_down++) << 8;
		
		r = saturate5((y_down_read + vr));
		g = saturate6((y_down_read - ug - vg));
		b = saturate5((y_down_read + ub));
		
		*dst_down++ = MAKE_RGBA_565(r, g, b);
		
		pixel--;
		if (pixel == 0)
		{
			// jump to next line
			y_buf_up += 2*src->y_woffset - line_size;
			y_buf_down += 2*src->y_woffset - line_size;
			cb_buf += src->c_woffset - (line_size>>1);
			cr_buf += src->c_woffset - (line_size>>1);
			
			dst_up += 2*dest_y_woffset - line_size;
			dst_down += 2*dest_y_woffset - line_size;
			
			pixel = line_size>>1;
			line--;
		}
	}
	
	return C_OK;
}

static C_RESULT
video_blockline_from_blockline_rgb24(video_picture_context_t* ctx,
                                     video_picture_context_t* src,
                                     int32_t num_macro_blocks)
{
  uint32_t y_up_read, y_down_read, cr_current, cb_current;
  int32_t u, v, vr, ug, vg, ub, r, g, b;
  uint8_t *y_buf_up, *y_buf_down, *cr_buf, *cb_buf;
  uint8_t *dst_up, *dst_down;
  int32_t line_size,dest_y_woffset;
  
  // Control variables
  uint32_t pixel,line;
  
  // In ptrs
  y_buf_up = src->y_src;
  y_buf_down = y_buf_up + src->y_woffset;
	
  cb_buf = src->cb_src;
  cr_buf = src->cr_src;
	
  dest_y_woffset = ctx->y_woffset;
	
  dst_up = (uint8_t*) ctx->y_src;
  dst_down = dst_up + dest_y_woffset;
	
  // We compute two pixels at a time
  line_size = MB_WIDTH_Y*num_macro_blocks; 
	
  pixel = line_size>>1;
  line = MB_WIDTH_Y>>1;
	
  while (line)
    {
      // load cb cr
      cb_current = *cb_buf++;
      cr_current = *cr_buf++;
		
      u = cb_current - 128;
      ug = 88 * u;
      ub = 454 * u;
      v = cr_current - 128;
      vg = 183 * v;
      vr = 359 * v;
		
      // compute pixel(0,0)
      y_up_read = (*y_buf_up++) << 8;
      r = saturate8((y_up_read + vr));
      g = saturate8((y_up_read - ug - vg));
      b = saturate8((y_up_read + ub));
		
      *dst_up++ = r; *dst_up++ = g; *dst_up++ = b;
		
      // compute pixel(1,0)
      y_up_read = (*y_buf_up++) << 8;
		
      r = saturate8((y_up_read + vr));
      g = saturate8((y_up_read - ug - vg));
      b = saturate8((y_up_read + ub));
		
      *dst_up++ = r; *dst_up++ = g; *dst_up++ = b;
		
      // compute pixel (0,1)
      y_down_read = (*y_buf_down++) << 8;
		
      r = saturate8((y_down_read + vr));
      g = saturate8((y_down_read - ug - vg));
      b = saturate8((y_down_read + ub));
		
      *dst_down++ = r; *dst_down++ = g; *dst_down++ = b;
		
      // compute pixel (1,1)
      y_down_read = (*y_buf_down++) << 8;
		
      r = saturate8((y_down_read + vr));
      g = saturate8((y_down_read - ug - vg));
      b = saturate8((y_down_read + ub));
		
      *dst_down++ = r; *dst_down++ = g; *dst_down++ = b;
		
      pixel--;
      if (pixel == 0)
        {
          // jump to next line
          y_buf_up += 2*src->y_woffset - line_size;
          y_buf_down += 2*src->y_woffset - line_size;
          cb_buf += src->c_woffset - (line_size>>1);
          cr_buf += src->c_woffset - (line_size>>1);
			
          dst_up += 2*dest_y_woffset - (line_size*3);
          dst_down += 2*dest_y_woffset - (line_size*3);
			
          pixel = line_size>>1;
          line--;
        }
    }
	
  return C_OK;
}
