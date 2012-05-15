#ifndef _VIDEO_GOB_H_
#define _VIDEO_GOB_H_

#include <VP_Os/vp_os_types.h>
#include <VLIB/video_macroblock.h>

//
// Description of a group of block compatible with the h263 standard
//  macroblocks is an array containing all the macroblocks of a blockline
//  quant is the default quantization value for the blockline
//
typedef struct _video_gob_t {
  video_macroblock_t* macroblocks;
  int32_t quant;
} video_gob_t;

#endif // _VIDEO_GOB_H_
