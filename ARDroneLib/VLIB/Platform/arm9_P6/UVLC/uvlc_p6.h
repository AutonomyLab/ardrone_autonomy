#ifndef _UVLC_P6_H_
#define _UVLC_P6_H_

#include <VLIB/UVLC/uvlc.h>

// not_last > 0 if this is not the last coefficient
// If last == 0, a special sequence is emitted
void     uvlc_encode( video_stream_t* const stream, int32_t level, int32_t run, int32_t not_last );

#endif // _UVLC_ENCODE_P5P_H_
