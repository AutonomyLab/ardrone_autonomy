#ifndef _P264_H_
#define _P264_H_

#include <VP_Os/vp_os_types.h>
#include <VLIB/video_controller.h>

// not_last > 0 if this is not the last coefficient
// If last == 0, a special sequence is emitted
void     p264_encode( video_stream_t* const stream, int32_t level, int32_t run, int32_t not_last );
C_RESULT p264_decode( video_stream_t* const stream, int32_t* run, uint32_t* level, int32_t* last);
void p264_encode_int(video_stream_t* const stream, int32_t code);
void p264_decode_int(video_stream_t* const stream, int32_t *code);
#endif // _P264_H_
