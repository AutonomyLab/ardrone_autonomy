#ifndef _VLIB_H_
#define _VLIB_H_

#include <VLIB/video_controller.h>

#define VLIB_DEFAULT_BITRATE          (0) /* In bytes/frame, 0 means bitrate control disabled*/

struct _video_codec_t {
  encode_blockline_fc encode_blockline;
  decode_blockline_fc decode_blockline;
  update_fc           update;
  cache_stream_fc     cache_stream;
};

/******** Available codecs ********/
typedef enum _codec_type_t {
  NULL_CODEC    = 0,
  UVLC_CODEC    = 0x20,       // codec_type value is used for START_CODE
  MJPEG_CODEC,                // not used
  P263_CODEC,                 // not used
  P264_CODEC    = 0x40,
  MP4_360P_CODEC = 0x80,
  H264_360P_CODEC = 0x81,
  MP4_360P_H264_720P_CODEC = 0x82,
  H264_720P_CODEC = 0x83,
  MP4_360P_SLRS_CODEC = 0x84,
  H264_360P_SLRS_CODEC = 0x85,
  H264_720P_SLRS_CODEC = 0x86,
  H264_AUTO_RESIZE_CODEC = 0x87,    // resolution is automatically adjusted according to bitrate
  MP4_360P_H264_360P_CODEC = 0x88,

} codec_type_t;

/******** API ********/
C_RESULT video_codec_open( video_controller_t* controller, codec_type_t codec_type );
C_RESULT video_codec_close( video_controller_t* controller );

// Encode/Decode a complete picture given
C_RESULT video_encode_picture( video_controller_t* controller, const vp_api_picture_t* picture, bool_t* got_image );
C_RESULT video_decode_picture( video_controller_t* controller, vp_api_picture_t* picture, video_stream_t* ex_stream, bool_t* got_image );

// Encode/Decode a blockline
static INLINE C_RESULT video_encode_blockline( video_controller_t* controller, const vp_api_picture_t* blockline, bool_t picture_complete )
{
  return controller->video_codec->encode_blockline( controller, blockline, picture_complete );
}

C_RESULT video_decode_blockline( video_controller_t* controller, vp_api_picture_t* blockline, bool_t* got_image );

#endif // _VLIB_H_
