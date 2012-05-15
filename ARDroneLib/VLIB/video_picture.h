#ifndef _VIDEO_PICTURE_H_
#define _VIDEO_PICTURE_H_

#include <VP_Os/vp_os_types.h>
#include <VP_Api/vp_api_picture.h>

#include <VLIB/video_picture_defines.h>

typedef struct _video_picture_context_t {
  uint8_t*  y_src;
  uint8_t*  cb_src;
  uint8_t*  cr_src;

  uint32_t  y_woffset; // = picture->y_line_size (in bytes)
  uint32_t  c_woffset; // = picture->cb_line_size (in bytes)
  uint32_t  y_hoffset; // = picture->y_line_size * MCU_HEIGHT (in bytes)

} video_picture_context_t;

// Transform picture in macro blocks
C_RESULT video_blockline_to_macro_blocks(video_picture_context_t* ctx, int16_t* macro_blocks, int32_t num_macro_blocks);

// Transform macro blocks in picture
C_RESULT video_blockline_from_macro_blocks(video_picture_context_t* ctx, int16_t* macro_blocks, int32_t num_macro_blocks, enum PixelFormat format);

// Transform macro blocks in picture
C_RESULT video_blockline_from_blockline(video_picture_context_t* ctx, video_picture_context_t* src, int32_t num_macro_blocks, enum PixelFormat format);


#endif // _VIDEO_PICTURE_H_
