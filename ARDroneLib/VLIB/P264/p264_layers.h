#ifndef _P264_LAYERS_H_
#define _P264_LAYERS_H_

#include <VP_Os/vp_os_types.h>
#include <VLIB/video_macroblock.h>

#define MAKE_START_CODE(gob)      ( P264_CODEC | (gob) )
#define PICTURE_START_CODE        MAKE_START_CODE(0)
#define PICTURE_END_CODE          MAKE_START_CODE(0x1F)

#define UVLC_FORMAT_CIF   1
#define UVLC_FORMAT_VGA   2
//
//#define UVLC_RESOLUTION_SUBQ  1 /* sub-QCIF */
//#define UVLC_RESOLUTION_Q     2 /* QCIF     */
//#define UVLC_RESOLUTION_1     3 /* CIF      */
//#define UVLC_RESOLUTION_4     4 /* 4-CIF    */
//#define UVLC_RESOLUTION_16    5 /* 16-CIF   */

typedef struct _p264_mb_layer_t {
  uint32_t desc;
  uint32_t dquant;
} p264_mb_layer_t;

typedef struct _p264_gob_layer_t {
  video_macroblock_t* macroblocks;
  uint32_t quant;
} p264_gob_layer_t;

typedef struct _p264_picture_layer_t {
  uint32_t format;
  uint32_t resolution;
  uint32_t picture_type;
  uint32_t quant;
  p264_gob_layer_t* gobs;
} p264_picture_layer_t;

C_RESULT p264_write_picture_layer( video_controller_t* controller, video_stream_t* stream );
C_RESULT p264_read_picture_layer( video_controller_t* controller, video_stream_t* stream );

C_RESULT p264_write_gob_layer( video_stream_t* stream, p264_gob_layer_t* gob );
C_RESULT p264_read_gob_layer( video_stream_t* stream, p264_gob_layer_t* gob );

C_RESULT p264_write_mb_layer(video_controller_t* controller, video_stream_t* stream, video_macroblock_t* mb, int32_t num_macro_blocks );
C_RESULT p264_read_mb_layer(video_controller_t* controller, video_stream_t* stream, video_gob_t*  gobs, uint32_t i_blockline, uint32_t i_mb);

#endif // _UVLC_LAYERS_H_
