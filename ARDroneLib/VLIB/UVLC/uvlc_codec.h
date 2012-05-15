#ifndef _UVLC_CODEC_H_
#define _UVLC_CODEC_H_

#include <VP_Os/vp_os_types.h>

#include <VLIB/video_codec.h>
#include "uvlc_layers.h"

typedef struct _uvlc_codec_t {
  // Compatibility with video_codec_t structure
  encode_blockline_fc encode_blockline;
  decode_blockline_fc decode_blockline;
  update_fc           update;
  cache_stream_fc     cache_stream;

  // Private data (see video source coding algorithm p.9)
  uvlc_picture_layer_t  picture_layer;
} uvlc_codec_t;

void uvlc_codec_alloc( video_controller_t* controller );
void uvlc_codec_free( video_controller_t* controller );

C_RESULT uvlc_pack_controller( video_controller_t* controller );
C_RESULT uvlc_unpack_controller( video_controller_t* controller );

C_RESULT uvlc_encode_blockline( video_controller_t* controller, const vp_api_picture_t* blockline, bool_t picture_complete );
C_RESULT uvlc_decode_blockline( video_controller_t* controller, vp_api_picture_t* picture, bool_t* got_image );
C_RESULT uvlc_update( video_controller_t* controller );
C_RESULT uvlc_cache( video_controller_t* controller, video_stream_t* ex_stream);

#endif // _UVLC_CODEC_H_
