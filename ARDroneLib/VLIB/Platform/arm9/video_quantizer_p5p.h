#ifndef _VIDEO_QUANTIZER_P5P_H_
#define _VIDEO_QUANTIZER_P5P_H_

#include <VLIB/video_quantizer.h>

int16_t* do_quantize_intra_mb(int16_t* ptr, int32_t invQuant, int32_t* last_ptr);

#endif // _VIDEO_QUANTIZER_P5P_H_
