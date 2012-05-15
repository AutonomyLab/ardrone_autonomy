#ifndef _UVLC_H_
#define _UVLC_H_

#include <VP_Os/vp_os_types.h>
#include <VLIB/video_controller.h>

// not_last > 0 if this is not the last coefficient
// If last == 0, a special sequence is emitted
void     uvlc_encode( video_stream_t* const stream, int32_t level, int32_t run, int32_t not_last );
C_RESULT uvlc_decode( video_stream_t* const stream, int32_t* run, int32_t* level, int32_t* last);

#endif // _UVLC_H_
