#ifndef _P263_CODEC_H_
#define _P263_CODEC_H_

#include <VLIB/video_codec.h>
#include "p263_layers.h"
#include "p263_huffman.h"

typedef struct _p263_codec_t {
  // Compatibility with video_codec_t structure
  encode_blockline_fc encode_blockline;
  decode_blockline_fc decode_blockline;
  update_fc           update;
  cache_stream_fc     cache_stream;

  // Private data (see video source coding algorithm p.9)
  p263_picture_layer_t  picture_layer;
  p263_mb_type_t*       mb_types;
  p263_cbpy_t*          cbpys;
} p263_codec_t;

void p263_codec_alloc( video_controller_t* controller );
void p263_codec_free( video_controller_t* controller );

C_RESULT p263_encode_blockline( video_controller_t* controller, const vp_api_picture_t* blockline, bool_t picture_complete );
C_RESULT p263_decode_blockline( video_controller_t* controller, vp_api_picture_t* picture, bool_t* got_image );
C_RESULT p263_update( video_controller_t* controller );
C_RESULT p263_cache( video_controller_t* controller, video_stream_t* ex_stream);

#endif // _P263_CODEC_H_
