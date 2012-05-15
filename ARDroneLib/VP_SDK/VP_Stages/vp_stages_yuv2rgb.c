/**
 *  @file   vp_stages_yuv2rgb.c
 *  @brief  VP Stages. YUV to RGB converter stage declaration
*/

#ifdef _INCLUDED_FOR_DOXYGEN_
#else // ! _INCLUDED_FOR_DOXYGEN_

///////////////////////////////////////////////
// INCLUDES

#include <VP_Stages/vp_stages_yuv2rgb.h>
#include <VP_Api/vp_api_config.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>

#endif // < _INCLUDED_FOR_DOXYGEN_


#ifdef USE_YUV2RGB_STRETCH

static C_RESULT vp_stages_yuv2rgb_open(vp_stages_yuv2rgb_config_t *cfg);
static C_RESULT vp_stages_yuv2rgb_close(vp_stages_yuv2rgb_config_t *cfg);
static void vp_stages_yuv2rgb_hresample(uint32_t sample0, uint32_t sample1, int8_t cov, int8_t* frac, uint32_t* line0, uint32_t* line1, int* dx);
static void vp_stages_yuv2rgb_hstretch(uint8_t* y_base, int32_t y_rbytes, uint8_t* cb_base, uint8_t* cr_base, uint32_t* line0, uint32_t* line1);
static void vp_stages_yuv2rgb_vstretch(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes, int8_t cov, int8_t* frac, uint32_t* line, int* dy);
static void vp_stages_yuv2rgb_flushline_565(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes, int y);


#define SRC_WIDTH   176
#define SRC_HEIGHT  144
#define DST_WIDTH   320
#define DST_HEIGHT  240

/******************************************************************************/
/*                                                                            */
/*          176*144 (YCbCr, 4:2:0) -> 320*240 (RGB-16bits, 5:6:5)             */
/*                                                                            */
/******************************************************************************/

static int8_t htable[SRC_WIDTH] =
{
  0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E,
  0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F,
  0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F,
  0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E,
  0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E,
  0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F,
  0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F,
  0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E,
  0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E,
  0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F,
  0x0E, 0x0F, 0x0E, 0x0F, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0E, 0x0F, 0x0F
};

static int8_t vtable[SRC_HEIGHT] =
{
  0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D,
  0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D,
  0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E,
  0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D,
  0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D,
  0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E,
  0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D,
  0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D,
  0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E, 0x0D, 0x0D, 0x0E
};

/******************************************************************************/

#endif  // < USE_YUV2RGB_STRETCH


/**
 *  @def      VP_STAGES_YUV2RGB_LIMIT(x)
 *  @brief    Macro used for ARGB32 saturation
 *
 *  Moves x in a limited range of values (between 0 and 0xff).
 */
//#define VP_STAGES_YUV2ARGB_LIMIT(x)  ( (x) > 0xffff ? 0xff : ( (x) <= 0xff ? 0 : ( (x) >> 8 ) ) )
#define VP_STAGES_YUV2ARGB_LIMIT(dst, x) \
  dst = x; \
  dst = (dst > 0 ? ( ((dst) >> 8) > 0xff ? 0xff : ((dst) >> 8) ) : 0);

/**
 *  @def      VP_STAGES_YUV2RGB_SAT5U(a)
 *  @brief    5 bits saturation
 */
#define VP_STAGES_YUV2RGB_SAT5U(a)  \
  if((a) < 0) (a) = 0;              \
  else if((a) > 0x1F) (a) = 0x1F;

/**
 *  @def      VP_STAGES_YUV2RGB_SAT6U(a)
 *  @brief    6 bits saturation
 */
#define VP_STAGES_YUV2RGB_SAT6U(a)  \
  if((a) < 0) (a) = 0;              \
  else if((a) > 0x3F) (a) = 0x3F;

/** Pointer to a YUV to RGB conversion function
 */
static vp_stages_YUV_to_RGB_t vp_stages_YUV_to_RGB;

#ifndef QCIF_TO_QVGA
/** YUV to RGB conversion functions
 */
  static void vp_stages_YUV420P_to_RGB565(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes);
  static void vp_stages_YUV420P_to_RGB24(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes);
  static void vp_stages_YUV420P_to_ARGB32(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes);
#else // ! QCIF_TO_QVGA
/** YUV to RGB conversion + resizing
 */
#ifdef USE_YUV2RGB_STRETCH
  static void vp_stages_YUV420P_to_RGB565_QCIF_to_QVGA_stretch(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes);
#else  // ! USE_YUV2RGB_STRETCH
  static void vp_stages_YUV420P_to_RGB565_QCIF_to_QVGA(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes);
  static void vp_stages_YUV420P_to_RGB24_QCIF_to_QVGA(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes);
  static void vp_stages_YUV420P_to_ARGB32_QCIF_to_QVGA(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes);
#endif  // < USE_YUV2RGB_STRETCH
#endif // < QCIF_TO_QVGA


#ifndef QCIF_TO_QVGA
static void vp_stages_YUV420P_to_RGB565(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes)
{
  uint32_t  w, width;
  uint32_t  h, height;

  width = picture->width >> 1;
  height = picture->height >> 1;

  for (h = 0; h < height; h++)
  {
    uint8_t*  s0;
    uint8_t*  s1;
    uint8_t*  s2;
    uint16_t* d0;
    uint16_t* d1;

    s0 = picture->y_buf  + (h * picture->y_line_size * 2);
    s1 = picture->cb_buf + (h * picture->cb_line_size);
    s2 = picture->cr_buf + (h * picture->cr_line_size);
    d0 = (uint16_t*) (dst + (h * dst_rbytes * 2));
    d1 = (uint16_t*) (dst + (h * dst_rbytes * 2) + dst_rbytes);

    for (w = 0; w < width; w++)
    {
      int32_t   y, cb, cr;
      int32_t   c_r, c_g, c_b;
      int16_t   r, g, b;

      cb = ((uint32_t) *(s1++)) - 0x80L;
      cr = ((uint32_t) *(s2++)) - 0x80L;
      c_r = cr * 359L;
      c_g = (cb * -88L) + (cr * -183L);
      c_b = cb * 454L;

      y = ((uint32_t) s0[0]) << 8;
      r = (int16_t) ((y + c_r + 0x000) >> 11);      VP_STAGES_YUV2RGB_SAT5U(r);
      g = (int16_t) ((y + c_g + 0x000) >> 10);      VP_STAGES_YUV2RGB_SAT6U(g);
      b = (int16_t) ((y + c_b + 0x000) >> 11);      VP_STAGES_YUV2RGB_SAT5U(b);
      d0[0] = (r << 11) | (g << 5) | (b << 0);

      y = ((uint32_t) s0[1]) << 8;
      r = (int16_t) ((y + c_r + 0x400) >> 11);      VP_STAGES_YUV2RGB_SAT5U(r);
      g = (int16_t) ((y + c_g + 0x200) >> 10);      VP_STAGES_YUV2RGB_SAT6U(g);
      b = (int16_t) ((y + c_b + 0x400) >> 11);      VP_STAGES_YUV2RGB_SAT5U(b);
      d0[1] = (r << 11) | (g << 5) | (b << 0);

      y = ((uint32_t) s0[picture->y_line_size + 0]) << 8;
      r = (int16_t) ((y + c_r + 0x600) >> 11);      VP_STAGES_YUV2RGB_SAT5U(r);
      g = (int16_t) ((y + c_g + 0x300) >> 10);      VP_STAGES_YUV2RGB_SAT6U(g);
      b = (int16_t) ((y + c_b + 0x600) >> 11);      VP_STAGES_YUV2RGB_SAT5U(b);
      d1[0] = (r << 11) | (g << 5) | (b << 0);

      y = ((uint32_t) s0[picture->y_line_size + 1]) << 8;
      r = (int16_t) ((y + c_r + 0x200) >> 11);      VP_STAGES_YUV2RGB_SAT5U(r);
      g = (int16_t) ((y + c_g + 0x100) >> 10);      VP_STAGES_YUV2RGB_SAT6U(g);
      b = (int16_t) ((y + c_b + 0x200) >> 11);      VP_STAGES_YUV2RGB_SAT5U(b);
      d1[1] = (r << 11) | (g << 5) | (b << 0);

      s0 += 2;
      d0 += 2;
      d1 += 2;
    }
  }
}
#endif // < QCIF_TO_QVGA


#ifndef QCIF_TO_QVGA
static void vp_stages_YUV420P_to_RGB24(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes)
{
  uint32_t width, height;
  int32_t line, col, linewidth;
  int32_t y, u, v, r, g, b;
  int32_t vr, ug, vg, ub;
  uint8_t *py, *pu, *pv;

  int32_t lineSz0 = picture->y_line_size;
  int32_t lineSz1 = picture->cb_line_size;
  int32_t lineSz2 = picture->cr_line_size;

  width = picture->width;
  height = picture->height;

  linewidth = width - (width >> 1);

  if(cfg->mode == VP_STAGES_YUV2RGB_MODE_UPSIDE_DOWN)
    {
      py = picture->y_buf+lineSz0*(height-1)+width-1;
      pu = picture->cb_buf+lineSz1*((height>>1)-1)+((width>>1)-1);
      pv = picture->cr_buf+lineSz2*((height>>1)-1)+((width>>1)-1);

      for (line = height-1; line >= 0; line--) {
	for (col = width-1; col >= 0; col--) {
	  y   = *py;
	  y   = y << 8;
	  u   = *pu - 128;
	  ug  = 88 * u;
	  ub  = 454 * u;
	  v   = *pv - 128;
	  vg  = 183 * v;
	  vr  = 359 * v;

	  VP_STAGES_YUV2ARGB_LIMIT(r, y +      vr);
	  VP_STAGES_YUV2ARGB_LIMIT(g, y - ug - vg);
	  VP_STAGES_YUV2ARGB_LIMIT(b, y + ub     );

	  *dst = r;
	  dst++;
	  *dst = g;
	  dst++;
	  *dst = b;
	  dst++;

	  py--;

	  if (col & 1)
	    {
	      pu--;
	      pv--;
	    } // No else
	}

	pu += linewidth;
	pv += linewidth;

	if (line & 1)
	  {
	    pu -= lineSz1;
	    pv -= lineSz2;
	  } // No else
	py -= lineSz0 - width;
      }
    }
  else
    {
      py = picture->y_buf;
      pu = picture->cb_buf;
      pv = picture->cr_buf;

      for (line = 0; line < (int32_t)height; line++) {
	for (col = 0; col < (int32_t)width; col++) {
	  y   = *py;
	  y   = y << 8;
	  u   = *pu - 128;
	  ug  = 88 * u;
	  ub  = 454 * u;
	  v   = *pv - 128;
	  vg  = 183 * v;
	  vr  = 359 * v;

	  VP_STAGES_YUV2ARGB_LIMIT(r, y +      vr);
	  VP_STAGES_YUV2ARGB_LIMIT(g, y - ug - vg);
	  VP_STAGES_YUV2ARGB_LIMIT(b, y + ub     );

	  *dst = r;
	  dst++;
	  *dst = g;
	  dst++;
	  *dst = b;
	  dst++;

	  py++;

	  if (col & 1)
	    {
	      pu++;
	      pv++;
	    } // No else
	}

	pu -= linewidth;
	pv -= linewidth;

	if (line & 1)
	  {
	    pu += lineSz1;
	    pv += lineSz2;
	  } // No else
	py += lineSz0 - width;
      }
    }
}
#endif // < QCIF_TO_QVGA


#ifndef QCIF_TO_QVGA
static void vp_stages_YUV420P_to_ARGB32(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes)
{
  uint32_t width, height;
  int32_t line, col, linewidth;
  int32_t y, u, v, r, g, b;
  int32_t vr, ug, vg, ub;
  uint8_t *py, *pu, *pv;

  int32_t lineSz0 = picture->y_line_size;
  int32_t lineSz1 = picture->cb_line_size;
  int32_t lineSz2 = picture->cr_line_size;

  width = picture->width;
  height = picture->height;

  linewidth = width - (width >> 1);

  py = picture->y_buf;
  pu = picture->cb_buf;
  pv = picture->cr_buf;

  for (line = 0; line < (int32_t)height; line++) {
    for (col = 0; col < (int32_t)width; col++) {
      y   = *py;
      y   = y << 8;
      u   = *pu - 128;
      ug  = 88 * u;
      ub  = 454 * u;
      v   = *pv - 128;
      vg  = 183 * v;
      vr  = 359 * v;

      VP_STAGES_YUV2ARGB_LIMIT(r, y + vr);
      VP_STAGES_YUV2ARGB_LIMIT(g, y - ug - vg);
      VP_STAGES_YUV2ARGB_LIMIT(b, y + ub);

      *dst = b;
      dst++;
      *dst = g;
      dst++;
      *dst = r;
      dst+=2;        //skip Alpha value

      py++;

      if (col & 1)
      {
        pu++;
        pv++;
      } // No else
    }

    pu -= linewidth;
    pv -= linewidth;

    if (line & 1)
    {
      pu += lineSz1;
      pv += lineSz2;
    } // No else
    py += lineSz0 - width;
  }
}
#endif // < QCIF_TO_QVGA


#ifdef QCIF_TO_QVGA
#ifdef USE_YUV2RGB_STRETCH
static void vp_stages_YUV420P_to_RGB565_QCIF_to_QVGA_stretch(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes)
{
  int     h;
  uint8_t*  y;
  uint8_t*  cb;
  uint8_t*  cr;
  int     dy;
  int8_t    vfrac;

  vp_os_memset(cfg->vline, 0, dst_rbytes);

  y = picture->y_buf;
  cb = picture->cb_buf;
  cr = picture->cr_buf;
  vfrac = 8;
  dy = 0;
  for (h = 0; h < SRC_HEIGHT; h += 2)
  {
    vp_stages_yuv2rgb_hstretch(y, picture->y_line_size, cb, cr, cfg->hline0, cfg->hline1);
    vp_stages_yuv2rgb_vstretch(cfg, picture, dst, dst_rbytes, vtable[h + 0], &vfrac, cfg->hline0, &dy);
    vp_stages_yuv2rgb_vstretch(cfg, picture, dst, dst_rbytes, vtable[h + 1], &vfrac, cfg->hline1, &dy);
    y += (picture->y_line_size * 2);
    cb += picture->cb_line_size;
    cr += picture->cr_line_size;
  }
}
#endif  // < USE_YUV2RGB_STRETCH
#endif  // <  QCIF_TO_QVGA


#ifdef QCIF_TO_QVGA
#ifndef USE_YUV2RGB_STRETCH
static void vp_stages_YUV420P_to_RGB565_QCIF_to_QVGA(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes)
{
  uint32_t  w, width;
  uint32_t  h, height;
  uint8_t*  s0;
  uint8_t*  s1;
  uint8_t*  s2;
  uint16_t* d0;
  uint16_t* d1;
  int32_t   y, cb, cr;
  int32_t   c_r, c_g, c_b;
  int16_t   r, g, b;

  width = QVGA_WIDTH >> 2;
  height = QVGA_HEIGHT >> 2;

  for (h = 0; h < height; h++)
  {
    s0 = picture->y_buf  + ((h * picture->y_line_size)<<1);
    s1 = picture->cb_buf + (h * picture->cb_line_size);
    s2 = picture->cr_buf + (h * picture->cr_line_size);
    d0 = (uint16_t*) (dst + ((h * QVGA_WIDTH) << 3));
    d1 = (uint16_t*) (dst + ((h * QVGA_WIDTH) << 3) + (QVGA_WIDTH << 2));

    for (w = 0; w < width; w++)
    {
      cb = ((uint32_t) *(s1++)) - 0x80L;
      cr = ((uint32_t) *(s2++)) - 0x80L;
      c_r = cr * 359L;
      c_g = (cb * -88L) + (cr * -183L);
      c_b = cb * 454L;

      y = ((uint32_t) s0[0]) << 8;
      r = (int16_t) ((y + c_r + 0x000) >> 11);      VP_STAGES_YUV2RGB_SAT5U(r);
      g = (int16_t) ((y + c_g + 0x000) >> 10);      VP_STAGES_YUV2RGB_SAT6U(g);
      b = (int16_t) ((y + c_b + 0x000) >> 11);      VP_STAGES_YUV2RGB_SAT5U(b);
      d0[0] = (r << 11) | (g << 5) | (b << 0);
      d0[1] = d0[0];

      y = ((uint32_t) s0[1]) << 8;
      r = (int16_t) ((y + c_r + 0x400) >> 11);      VP_STAGES_YUV2RGB_SAT5U(r);
      g = (int16_t) ((y + c_g + 0x200) >> 10);      VP_STAGES_YUV2RGB_SAT6U(g);
      b = (int16_t) ((y + c_b + 0x400) >> 11);      VP_STAGES_YUV2RGB_SAT5U(b);
      d0[2] = (r << 11) | (g << 5) | (b << 0);
      d0[3] = d0[2];

      y = ((uint32_t) s0[picture->y_line_size + 0]) << 8;
      r = (int16_t) ((y + c_r + 0x600) >> 11);      VP_STAGES_YUV2RGB_SAT5U(r);
      g = (int16_t) ((y + c_g + 0x300) >> 10);      VP_STAGES_YUV2RGB_SAT6U(g);
      b = (int16_t) ((y + c_b + 0x600) >> 11);      VP_STAGES_YUV2RGB_SAT5U(b);
      d1[0] = (r << 11) | (g << 5) | (b << 0);
      d1[1] = d1[0];

      y = ((uint32_t) s0[picture->y_line_size + 1]) << 8;
      r = (int16_t) ((y + c_r + 0x200) >> 11);      VP_STAGES_YUV2RGB_SAT5U(r);
      g = (int16_t) ((y + c_g + 0x100) >> 10);      VP_STAGES_YUV2RGB_SAT6U(g);
      b = (int16_t) ((y + c_b + 0x200) >> 11);      VP_STAGES_YUV2RGB_SAT5U(b);
      d1[2] = (r << 11) | (g << 5) | (b << 0);
      d1[3] = d1[2];

      s0 += 2;
      d0 += 4;
      d1 += 4;
    }
    vp_os_memcpy(dst + ((h * QVGA_WIDTH) << 3) + (QVGA_WIDTH<<1),    dst + ((h * QVGA_WIDTH) << 3),                    QVGA_WIDTH<<1);
    vp_os_memcpy(dst + ((h * QVGA_WIDTH) << 3) + (QVGA_WIDTH<<1)*3,  dst + ((h * QVGA_WIDTH) << 3) + (QVGA_WIDTH<<2),  QVGA_WIDTH<<1);
  }
}
#endif  // < USE_YUV2RGB_STRETCH
#endif  // <  QCIF_TO_QVGA


#ifdef QCIF_TO_QVGA
#ifndef USE_YUV2RGB_STRETCH
static void vp_stages_YUV420P_to_RGB24_QCIF_to_QVGA(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes)
{
}
#endif  // < USE_YUV2RGB_STRETCH
#endif  // <  QCIF_TO_QVGA


#ifdef QCIF_TO_QVGA
#ifndef USE_YUV2RGB_STRETCH
static void vp_stages_YUV420P_to_ARGB32_QCIF_to_QVGA(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes)
{
  uint32_t width, height;
  int32_t line, col, linewidth;
  int32_t y, u, v, r, g, b;
  int32_t vr, ug, vg, ub;
  uint8_t *py, *pu, *pv;

  int32_t lineSz0 = picture->y_line_size;
  int32_t lineSz1 = picture->cb_line_size;
  int32_t lineSz2 = picture->cr_line_size;

  width = picture->width;
  height = picture->height;

  linewidth = width - (width >> 1);

  py = picture->y_buf;
  pu = picture->cb_buf;
  pv = picture->cr_buf;

  for (line = 0; line < (QVGA_HEIGHT >> 1); line++) {
    for (col = 0; col < (QVGA_WIDTH >> 1); col++) {
      y   = *py;
      y   = y << 8;
      u   = *pu - 128;
      ug  = 88 * u;
      ub  = 454 * u;
      v   = *pv - 128;
      vg  = 183 * v;
      vr  = 359 * v;

      VP_STAGES_YUV2ARGB_LIMIT(r, y + vr);
      VP_STAGES_YUV2ARGB_LIMIT(g, y - ug - vg);
      VP_STAGES_YUV2ARGB_LIMIT(b, y + ub);

      *dst = b;
      dst++;
      *dst = g;
      dst++;
      *dst = r;
      dst+=2;        //skip Alpha value
      *dst = b;
      dst++;
      *dst = g;
      dst++;
      *dst = r;
      dst+=2;

      py++;

      if (col & 1)
      {
        pu++;
        pv++;
      } // No else
    }

    pu -= (QVGA_WIDTH >> 2);
    pv -= (QVGA_WIDTH >> 2);

    if (line & 1)
    {
      pu += lineSz1;
      pv += lineSz2;
    } // No else
    py += lineSz0 - (QVGA_WIDTH >> 1);

    vp_os_memcpy(dst, dst - (QVGA_WIDTH<<2), QVGA_WIDTH<<2);
    dst += (QVGA_WIDTH<<2);
  }
}
#endif  // < USE_YUV2RGB_STRETCH
#endif  // <  QCIF_TO_QVGA


/******************************************************************************/

#ifdef USE_YUV2RGB_STRETCH
static C_RESULT vp_stages_yuv2rgb_open(vp_stages_yuv2rgb_config_t *cfg)
{
  C_RESULT res = VP_SUCCESS;

  cfg->hline0 = (uint32_t*) vp_os_malloc(DST_WIDTH * sizeof(uint32_t));
  cfg->hline1 = (uint32_t*) vp_os_malloc(DST_WIDTH * sizeof(uint32_t));
  cfg->vline  = (uint32_t*) vp_os_malloc(DST_WIDTH * sizeof(uint32_t));
  if (cfg->hline0 && cfg->hline1 && cfg->vline)
  {
//    vfrac = 8;
  }
  else
  {
    vp_stages_yuv2rgb_close(cfg);
    res = VP_FAILURE;
  }

  return res;
}

/******************************************************************************/

static C_RESULT vp_stages_yuv2rgb_close(vp_stages_yuv2rgb_config_t *cfg)
{
  if (cfg->hline0)
  {
    vp_os_free(cfg->hline0);
    cfg->hline0 = NULL;
  }
  if (cfg->hline1)
  {
    vp_os_free(cfg->hline1);
    cfg->hline1 = NULL;
  }
  if (cfg->vline)
  {
    vp_os_free(cfg->vline);
    cfg->vline = NULL;
  }
  return(VP_SUCCESS);
}

/******************************************************************************/

static void vp_stages_yuv2rgb_hresample(uint32_t sample0, uint32_t sample1, int8_t cov, int8_t* frac, uint32_t* line0, uint32_t* line1, int* dx)
{
  int8_t  f;
  int   i;

  f = *frac;
  i = *dx;
  do
  {
    int8_t  c;

    if (f < cov)
    {
      c = f;
    }
    else
    {
      c = cov;
    }
    cov -= c;
    f -= c;
    switch (c)
    {
      case 0:
      assert(0);
      break;

      case 1:
      line0[i] += (sample0 >> 3) & 0x1F1F1F1FUL;
      line1[i] += (sample1 >> 3) & 0x1F1F1F1FUL;
      break;

      case 2:
      line0[i] += (sample0 >> 2) & 0x3F3F3F3FUL;
      line1[i] += (sample1 >> 2) & 0x3F3F3F3FUL;
      break;

      case 3:
      line0[i] += ((sample0 >> 1) & 0x7F7F7F7FUL) - ((sample0 >> 3) & 0x1F1F1F1FUL);
      line1[i] += ((sample1 >> 1) & 0x7F7F7F7FUL) - ((sample1 >> 3) & 0x1F1F1F1FUL);
      break;

      case 4:
      line0[i] += (sample0 >> 1) & 0x7F7F7F7FUL;
      line1[i] += (sample1 >> 1) & 0x7F7F7F7FUL;
      break;

      case 5:
      line0[i] += ((sample0 >> 1) & 0x7F7F7F7FUL) + ((sample0 >> 3) & 0x1F1F1F1FUL);
      line1[i] += ((sample1 >> 1) & 0x7F7F7F7FUL) + ((sample1 >> 3) & 0x1F1F1F1FUL);
      break;

      case 6:
      line0[i] += sample0 - ((sample0 >> 2) & 0x3F3F3F3FUL);
      line1[i] += sample1 - ((sample1 >> 2) & 0x3F3F3F3FUL);
      break;

      case 7:
      line0[i] += sample0 - ((sample0 >> 3) & 0x1F1F1F1FUL);
      line1[i] += sample1 - ((sample1 >> 3) & 0x1F1F1F1FUL);
      break;

      case 8:
      line0[i] += sample0;
      line1[i] += sample1;
      break;
    }

    if (f == 0)
    {
      f = 8;
      i++;
      if (i < DST_WIDTH)
      {
        line0[i] = 0UL;
        line1[i] = 0UL;
      }
    }
  } while (cov);
  *dx = i;
  *frac = f;
}

/*----------------------------------------------------------------------------*/

static void vp_stages_yuv2rgb_hstretch(uint8_t* y_base, int32_t y_rbytes, uint8_t* cb_base, uint8_t* cr_base, uint32_t* line0, uint32_t* line1)
{
  int     sx, dx;
  int8_t    frac;
  uint8_t*  s0;
  uint8_t*  s1;
  uint8_t*  s2;

  frac = 8;
  dx = 0;
  line0[0] = 0UL;
  line1[0] = 0UL;
  s0 = y_base;
  s1 = cb_base;
  s2 = cr_base;
  for (sx = 0; sx < 176; sx += 2)
  {
    int32_t   cb, cr;
    uint32_t  cr_cg_cb, smp0, smp1;

    cb = (int32_t) ((uint32_t) *(s1++));
    cr = (int32_t) ((uint32_t) *(s2++));
    cr_cg_cb = ((cr * 180L) << 8) & 0xFF0000UL;
    cr_cg_cb |= ((cb * 44L) + (cr * 91L)) & 0xFF00UL;
    cr_cg_cb |= ((cb * 227L) >> 8) & 0xFFUL;

    smp0 = cr_cg_cb | (((uint32_t) s0[0]) << 24);
    smp1 = cr_cg_cb | (((uint32_t) s0[y_rbytes + 0]) << 24);
    vp_stages_yuv2rgb_hresample(smp0, smp1, htable[sx], &frac, line0, line1, &dx);

    smp0 = cr_cg_cb | (((uint32_t) s0[1]) << 24);
    smp1 = cr_cg_cb | (((uint32_t) s0[y_rbytes + 1]) << 24);
    vp_stages_yuv2rgb_hresample(smp0, smp1, htable[sx], &frac, line0, line1, &dx);
    s0 += 2;
  }
}

/*----------------------------------------------------------------------------*/

static void vp_stages_yuv2rgb_vstretch(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst_base, uint32_t dst_rbytes, int8_t cov, int8_t* frac, uint32_t* line, int* dy)
{
  int8_t    f;
  uint32_t* src;
  uint32_t* dst;
  int     y;

  f = *frac;
  y = *dy;
  do
  {
    int8_t  c;

    if (f < cov)
    {
      c = f;
    }
    else
    {
      c = cov;
    }
    cov -= c;
    f -= c;
    src = line;
    dst = cfg->vline;
    switch (c)
    {
      int   dx;

      case 8:
      for (dx = 0; dx < DST_WIDTH; dx++)
      {
        uint32_t  ycc;

        ycc = src[dx];
        dst[dx] = ycc;
      }
      break;

      case 7:
      for (dx = 0; dx < DST_WIDTH; dx++)
      {
        uint32_t  ycc;

        ycc = src[dx];
        dst[dx] += (ycc) - ((ycc >> 3) & 0x1F1F1F1FUL);
      }
      break;

      case 6:
      for (dx = 0; dx < DST_WIDTH; dx++)
      {
        uint32_t  ycc;

        ycc = src[dx];
        dst[dx] += (ycc) - ((ycc >> 2) & 0x3F3F3F3FUL);
      }
      break;

      case 5:
      for (dx = 0; dx < DST_WIDTH; dx++)
      {
        uint32_t  ycc;

        ycc = src[dx];
        dst[dx] += ((ycc >> 1) & 0x7F7F7F7FUL) + ((ycc >> 3) & 0x1F1F1F1FUL);
      }
      break;

      case 4:
      for (dx = 0; dx < DST_WIDTH; dx++)
      {
        uint32_t  ycc;

        ycc = src[dx];
        dst[dx] += ((ycc >> 1) & 0x7F7F7F7FUL);
      }
      break;

      case 3:
      for (dx = 0; dx < DST_WIDTH; dx++)
      {
        uint32_t  ycc;

        ycc = src[dx];
        dst[dx] += ((ycc >> 1) & 0x7F7F7F7FUL) - ((ycc >> 3) & 0x1F1F1F1FUL);
      }
      break;

      case 2:
      for (dx = 0; dx < DST_WIDTH; dx++)
      {
        uint32_t  ycc;

        ycc = src[dx];
        dst[dx] += ((ycc >> 2) & 0x3F3F3F3FUL);
      }
      break;

      case 1:
      for (dx = 0; dx < DST_WIDTH; dx++)
      {
        uint32_t  ycc;

        ycc = src[dx];
        dst[dx] += ((ycc >> 3) & 0x1F1F1F1FUL);
      }
      break;

      case 0:
      assert(0);
      break;
    }

    if (f == 0)
    {
      vp_stages_yuv2rgb_flushline_565(cfg, picture, dst_base, dst_rbytes, y++);
      f = 8;
    }
  } while (cov);

  *dy = y;
  *frac = f;
}

/*----------------------------------------------------------------------------*/

static void vp_stages_yuv2rgb_flushline_565(vp_stages_yuv2rgb_config_t *cfg, vp_api_picture_t *picture, uint8_t *dst, uint32_t dst_rbytes, int y)
{
#ifdef __linux__
  uint32_t* s;
  uint16_t* d;
  int     i, even, odd;

  s = cfg->vline;
  d = (uint16_t*) (dst + (dst_rbytes * y));
  even = 0;
  odd = 4;
  if (y & 1)
  {
    even ^= 6;
    odd ^= 6;
  }
  for (i = 0; i < DST_WIDTH; i += 2)
  {
    int32_t   ycc;
    int32_t   y;
    int32_t   r, g, b;
    int32_t   c_r, c_g, c_b;

    ycc = s[0];
    y = ((uint32_t) ycc) >> 24;
    c_r = ycc << 8;           c_r = ((uint32_t) c_r) >> 23;   c_r  -= 180;
    c_g = ycc << 16;          c_g = ((uint32_t) c_g) >> 23;   c_g  -= 135;
    c_b = ycc << 24;          c_b = ((uint32_t) c_b) >> 23;   c_b  -= 227;
    r = (even + y + c_r) >> 3;      VP_STAGES_YUV2RGB_SAT5U(r);
    g = ((even >> 1) + y - c_g) >> 2; VP_STAGES_YUV2RGB_SAT6U(g);
    b = (even + y + c_b) >> 3;      VP_STAGES_YUV2RGB_SAT5U(b);
    d[0] = (r << 11) | (g << 5) | (b << 0);

    ycc = s[1];
    y = ((uint32_t) ycc) >> 24;
    c_r = ycc << 8;           c_r = ((uint32_t) c_r) >> 23;   c_r  -= 180;
    c_g = ycc << 16;          c_g = ((uint32_t) c_g) >> 23;   c_g  -= 135;
    c_b = ycc << 24;          c_b = ((uint32_t) c_b) >> 23;   c_b  -= 227;
    r = (odd + y + c_r) >> 3;     VP_STAGES_YUV2RGB_SAT5U(r);
    g = ((odd >> 1) + y - c_g) >> 2;  VP_STAGES_YUV2RGB_SAT6U(g);
    b = (odd + y + c_b) >> 3;     VP_STAGES_YUV2RGB_SAT5U(b);
    d[1] = (r << 11) | (g << 5) | (b << 0);

    s[0] = 0U;
    s[1] = 0U;
    s += 2;
    d += 2;
  }
#else
  asm_yuvrgb565(dst, dst_rbytes, cfg->vline, y);
//static void flushline_565(uint16_t* dst, int32_t dst_rbytes, uint32_t* src, int y);
#endif
}

/******************************************************************************/
#endif  // < USE_YUV2RGB_STRETCH


C_RESULT vp_stages_yuv2rgb_stage_open(vp_stages_yuv2rgb_config_t *cfg)
{
  C_RESULT res = VP_SUCCESS;

#ifdef USE_YUV2RGB_STRETCH
  if(VP_FAILED(vp_stages_yuv2rgb_open(cfg))) {
    res = VP_FAILURE;
  }
#endif  // < USE_YUV2RGB_STRETCH

  return res;
}


C_RESULT vp_stages_yuv2rgb_stage_transform(vp_stages_yuv2rgb_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  uint32_t width, height;
  static uint32_t bytesPerPixel = 0;
  uint8_t *dst;
  vp_api_picture_t *picture;

  vp_os_mutex_lock(&out->lock);

  if( in->size > 0 )
  {
    picture = (vp_api_picture_t *) in->buffers; //&in->buffers[in->indexBuffer];

#ifndef QCIF_TO_QVGA
    width = picture->width;
    height = picture->height;
#else   // QCIF_TO_QVGA
    width = QVGA_WIDTH;
    height = QVGA_HEIGHT;
#endif  // <  QCIF_TO_QVGA

    if(out->status == VP_API_STATUS_INIT)
    {
      switch(cfg->rgb_format)
      {
        case VP_STAGES_RGB_FORMAT_RGB565:
          bytesPerPixel = 2;
#ifndef QCIF_TO_QVGA
          vp_stages_YUV_to_RGB = vp_stages_YUV420P_to_RGB565;
#else   // QCIF_TO_QVGA
# ifndef USE_YUV2RGB_STRETCH
          vp_stages_YUV_to_RGB = vp_stages_YUV420P_to_RGB565_QCIF_to_QVGA;
# else   // USE_YUV2RGB_STRETCH
          vp_stages_YUV_to_RGB = vp_stages_YUV420P_to_RGB565_QCIF_to_QVGA_stretch;
# endif  // <  USE_YUV2RGB_STRETCH
#endif  // <  QCIF_TO_QVGA
          break;

        case VP_STAGES_RGB_FORMAT_RGB24:
          bytesPerPixel = 3;
#ifndef QCIF_TO_QVGA
          vp_stages_YUV_to_RGB = vp_stages_YUV420P_to_RGB24;
#else   // QCIF_TO_QVGA
          vp_stages_YUV_to_RGB = vp_stages_YUV420P_to_RGB24_QCIF_to_QVGA;
#endif  // <  QCIF_TO_QVGA
          break;

        case VP_STAGES_RGB_FORMAT_ARGB32:
          bytesPerPixel = 4;
#ifndef QCIF_TO_QVGA
          vp_stages_YUV_to_RGB = vp_stages_YUV420P_to_ARGB32;
#else   // QCIF_TO_QVGA
          vp_stages_YUV_to_RGB = vp_stages_YUV420P_to_ARGB32_QCIF_to_QVGA;
#endif  // <  QCIF_TO_QVGA
          break;

        default :
          vp_stages_YUV_to_RGB = NULL;
          break;
      }

      VP_OS_ASSERT(vp_stages_YUV_to_RGB != NULL);
      VP_OS_ASSERT(bytesPerPixel != 0);

      out->numBuffers   = 1;
      out->indexBuffer  = 0;
      out->size         = width * height * bytesPerPixel;
      out->buffers      = (int8_t **) vp_os_malloc(sizeof(int8_t *)+out->size*sizeof(int8_t));
      out->buffers[out->indexBuffer] = (int8_t *)(out->buffers+1);
      out->lineSize     = (int32_t *) vp_os_malloc(out->numBuffers * sizeof(int32_t *));
      out->lineSize[out->indexBuffer] = width*bytesPerPixel;
      vp_os_memset(out->buffers[out->indexBuffer], 0, out->size);

      out->status = VP_API_STATUS_PROCESSING;
    } // No else

    if( out->status == VP_API_STATUS_PROCESSING )
    {
      dst = (uint8_t*)out->buffers[out->indexBuffer];

      vp_stages_YUV_to_RGB(cfg, picture, dst, width * bytesPerPixel);
      out->size = width * height * bytesPerPixel;
    } // No else
  }
  else
  {
    out->size = 0;
  }

  if(in->status == VP_API_STATUS_STILL_RUNNING) {
    out->status = VP_API_STATUS_PROCESSING;
  }
  else {
    out->status = in->status;
  }

  vp_os_mutex_unlock(&out->lock);

  return (VP_SUCCESS);
}


C_RESULT vp_stages_yuv2rgb_stage_close(vp_stages_yuv2rgb_config_t *cfg)
{
  C_RESULT res = VP_SUCCESS;

#ifdef USE_YUV2RGB_STRETCH
  if(VP_FAILED(vp_stages_yuv2rgb_close(cfg))) {
    res = VP_FAILURE;
  }
#endif  // < USE_YUV2RGB_STRETCH

  return res;
}
