
#include <VP_Os/vp_os_types.h>

#include <VLIB/video_codec.h>
#include "p264_layers.h"

typedef struct _p264_codec_t {
  // Compatibility with video_codec_t structure
  encode_blockline_fc encode_blockline;
  decode_blockline_fc decode_blockline;
  update_fc           update;
  cache_stream_fc     cache_stream;

  // Private data (see video source coding algorithm p.9)
  p264_picture_layer_t  picture_layer;
  vp_api_picture_t ref_picture;      // contains the reference picture used to decode inter frames
  vp_api_picture_t decoded_picture;  // contains the current decoded picture
  uint32_t         ip_counter;       // counter used to switch between P&I frames
  uint32_t         last_I_size;
  uint32_t         last_P_size;
} p264_codec_t;

void p264_codec_alloc( video_controller_t* controller );
void p264_codec_free( video_controller_t* controller );

C_RESULT p264_pack_controller( video_controller_t* controller );
C_RESULT p264_unpack_controller( video_controller_t* controller );

C_RESULT p264_encode_blockline( video_controller_t* controller, const vp_api_picture_t* blockline, bool_t picture_complete );
C_RESULT p264_decode_blockline( video_controller_t* controller, vp_api_picture_t* picture, bool_t* got_image );
C_RESULT p264_update( video_controller_t* controller );
C_RESULT p264_cache( video_controller_t* controller, video_stream_t* ex_stream);
