/**
 *  \brief    VP Api. Output SDL stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  28/03/2007
 */

#ifndef _VP_API_PICTURE_H_
#define _VP_API_PICTURE_H_

#include <VP_Os/vp_os_types.h>
#ifdef FFMPEG_SUPPORT
#include <libavutil/avutil.h>
#else
/**
 * Pixel format. Notes:
 *
 * PIX_FMT_RGBA32 is handled in an endian-specific manner. A RGBA
 * color is put together as:
 *  (A << 24) | (R << 16) | (G << 8) | B
 * This is stored as BGRA on little endian CPU architectures and ARGB on
 * big endian CPUs.
 *
 * When the pixel format is palettized RGB (PIX_FMT_PAL8), the palettized
 * image data is stored in AVFrame.data[0]. The palette is transported in
 * AVFrame.data[1] and, is 1024 bytes long (256 4-byte entries) and is
 * formatted the same as in PIX_FMT_RGBA32 described above (i.e., it is
 * also endian-specific). Note also that the individual RGB palette
 * components stored in AVFrame.data[1] should be in the range 0..255.
 * This is important as many custom PAL8 video codecs that were designed
 * to run on the IBM VGA graphics adapter use 6-bit palette components.
 */
enum PixelFormat {
    PIX_FMT_NONE= -1,
    PIX_FMT_YUV420P,   ///< Planar YUV 4:2:0 (1 Cr & Cb sample per 2x2 Y samples)
    PIX_FMT_YUV422,    ///< Packed pixel, Y0 Cb Y1 Cr
    PIX_FMT_RGB24,     ///< Packed pixel, 3 bytes per pixel, RGBRGB...
    PIX_FMT_BGR24,     ///< Packed pixel, 3 bytes per pixel, BGRBGR...
    PIX_FMT_YUV422P,   ///< Planar YUV 4:2:2 (1 Cr & Cb sample per 2x1 Y samples)
    PIX_FMT_YUV444P,   ///< Planar YUV 4:4:4 (1 Cr & Cb sample per 1x1 Y samples)
    PIX_FMT_RGBA32,    ///< Packed pixel, 4 bytes per pixel, BGRABGRA..., stored in cpu endianness
    PIX_FMT_YUV410P,   ///< Planar YUV 4:1:0 (1 Cr & Cb sample per 4x4 Y samples)
    PIX_FMT_YUV411P,   ///< Planar YUV 4:1:1 (1 Cr & Cb sample per 4x1 Y samples)
    PIX_FMT_RGB565,    ///< always stored in cpu endianness
    PIX_FMT_RGB555,    ///< always stored in cpu endianness, most significant bit to 1
    PIX_FMT_GRAY8,
    PIX_FMT_MONOWHITE, ///< 0 is white
    PIX_FMT_MONOBLACK, ///< 0 is black
    PIX_FMT_PAL8,      ///< 8 bit with RGBA palette
    PIX_FMT_YUVJ420P,  ///< Planar YUV 4:2:0 full scale (jpeg)
    PIX_FMT_YUVJ422P,  ///< Planar YUV 4:2:2 full scale (jpeg)
    PIX_FMT_YUVJ444P,  ///< Planar YUV 4:4:4 full scale (jpeg)
    PIX_FMT_XVMC_MPEG2_MC,///< XVideo Motion Acceleration via common packet passing(xvmc_render.h)
    PIX_FMT_XVMC_MPEG2_IDCT,
    PIX_FMT_UYVY422,   ///< Packed pixel, Cb Y0 Cr Y1
    PIX_FMT_UYVY411,   ///< Packed pixel, Cb Y0 Y1 Cr Y2 Y3
    PIX_FMT_VYUY422,   ///< Packed pixel, Cr Y0 Cb Y1
    PIX_FMT_NB,
};
#endif

#define OVERPAD                 16
#define MB_WIDTH_Y              16
#define MB_HEIGHT_Y             MB_WIDTH_Y
#define MB_WIDTH_C              8
#define MB_HEIGHT_C             MB_WIDTH_C


// SQCIF
#define SQCIF_WIDTH             128
#define SQCIF_HEIGHT            96
#define SQCIF_SIZE              (SQCIF_WIDTH * SQCIF_HEIGHT)

// QCIF
#define QCIF_WIDTH              176
#define QCIF_HEIGHT             144
#define QCIF_SIZE               (QCIF_WIDTH * QCIF_HEIGHT)

#define QQVGA_WIDTH             160
#define QQVGA_HEIGHT            120
#define QQVGA_SIZE              (QQVGA_WIDTH * QQVGA_HEIGHT)

// QQCIF
#define QQCIF_WIDTH             88
#define QQCIF_HEIGHT            72
#define QQCIF_SIZE              (QQCIF_WIDTH * QQCIF_HEIGHT)

// QQVGA
#define QQVGA_WIDTH             160
#define QQVGA_HEIGHT            120
#define QQVGA_SIZE              (QQVGA_WIDTH * QQVGA_HEIGHT)

// QVGA
#define QVGA_WIDTH              320
#define QVGA_HEIGHT             240
#define QVGA_SIZE               (QVGA_WIDTH * QVGA_HEIGHT)

// TWEAKY QQVGA
#define TWEAKY_QQVGA_WIDTH      320
#define TWEAKY_QQVGA_HEIGHT     (240-16)
#define TWEAKY_QQVGA_SIZE       (TWEAKY_QQVGA_WIDTH * TWEAKY_QQVGA_HEIGHT)

// CIF
#define CIF_WIDTH               352
#define CIF_HEIGHT              288
#define CIF_SIZE                (CIF_WIDTH * CIF_HEIGHT)

// VGA
#define VGA_WIDTH               640
#define VGA_HEIGHT              480
#define VGA_SIZE                (VGA_WIDTH * VGA_HEIGHT)


//360P
#define hdtv360P_WIDTH				640
#define hdtv360P_HEIGHT				360
#define hdtv360P_SIZE				( (hdtv360P_WIDTH) * (hdtv360P_HEIGHT) )

//720P
#define hdtv720P_WIDTH				1280
#define hdtv720P_HEIGHT				720
#define hdtv720P_SIZE				( (hdtv720P_WIDTH) * (hdtv720P_HEIGHT) )


#if defined(_MSC_VER)
	#define _ATTRIBUTE_PACKED_
	/* Asks Visual C++ to pack structures from now on*/
	#pragma pack(1)
#else
	#define _ATTRIBUTE_PACKED_  __attribute__ ((packed))
#endif


typedef struct _ATTRIBUTE_PACKED_ _vp_api_picture_
{
  enum PixelFormat format;    // camif -> encoder : PIX_FMT_YUV420P

  uint32_t         width;     // camif -> encoder
  uint32_t         height;    // camif -> encoder
  uint32_t         framerate; // camif -> encoder

  uint8_t         *raw;       // point to the start of the Y/Cb/Cr data block
  uint8_t         *y_buf;     // point to 1st Y  component in raw
  uint8_t         *cb_buf;    // point to 1st Cb component in raw
  uint8_t         *cr_buf;    // point to 1st Cr component in raw

  uint32_t         y_pad;     // 2* camif_config.y_pad
  uint32_t         c_pad;     // 2* camif_config.c_pad

  uint32_t         y_line_size;
  uint32_t         cb_line_size;
  uint32_t         cr_line_size;

  uint32_t         vision_complete;
  uint32_t         complete;
  int32_t          blockline;
  
  #ifdef USE_ELINUX
  uint32_t acquisition_timestamp;
  uint32_t scale;
  #endif
}
vp_api_picture_t;

C_RESULT vp_api_picture_print_allocator_log();
C_RESULT vp_api_picture_alloc(vp_api_picture_t * pic,const int width,const int height,enum PixelFormat format);
int vp_api_picture_get_buffer_size(const vp_api_picture_t * pic);
const char * vp_api_picture_get_filename_extension(const vp_api_picture_t * pic);
const char * vp_api_picture_get_pixelformat_name(enum PixelFormat fmt);
int vp_api_picture_format_to_buf_address(vp_api_picture_t * pic);
int vp_api_picture_point_to_buf_address(vp_api_picture_t * pic,void*addr);

#endif // ! _VP_API_PICTURE_H_
