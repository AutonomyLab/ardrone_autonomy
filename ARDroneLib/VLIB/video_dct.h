#ifndef _VIDEO_DCT_H_
#define _VIDEO_DCT_H_

#include <VLIB/video_picture.h>

#define NUM_MAX_DCT_BLOCKS  64U  /* Max number of blocks per dct calls */

// Default implementation for dct computation
void fdct(const unsigned short* in, short* out);
void idct(const short* in, unsigned short* out);

int16_t* video_fdct_quant_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks, int32_t quant);
int16_t* video_fdct_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks);
int16_t* video_idct_compute(int16_t* in, int16_t* out, int32_t num_macro_blocks);

#endif // _VIDEO_DCT_H_
