#ifndef _VIDEO_MACROBLOCK_H_
#define _VIDEO_MACROBLOCK_H_

#include <VP_Os/vp_os_types.h>
#include <VLIB/video_picture.h>
#include <VLIB/P264/p264_common.h>

// Default zigzag ordering matrix
extern int32_t video_zztable_t81[MCU_BLOCK_SIZE];

typedef struct _video_macroblock_t {
  int32_t   azq;  // All zero coefficients
  int32_t   dquant;
  int32_t   num_coeff_y0; // Number of non-zeros coefficients for block y0
  int32_t   num_coeff_y1; // Number of non-zeros coefficients for block y1
  int32_t   num_coeff_y2; // Number of non-zeros coefficients for block y2
  int32_t   num_coeff_y3; // Number of non-zeros coefficients for block y3
  int32_t   num_coeff_cb; // Number of non-zeros coefficients for block cb
  int32_t   num_coeff_cr; // Number of non-zeros coefficients for block cr

  // p264 additional data
  // --> intra data
  intra_type_t intra_type;
  intra_4x4_mode_t intra_4x4_mode[16]; // when encoding, intra_4x4_mode[] use a special format (see P6 h264) to handle intra prediction. when decoding intra_4x4_mode[] contains the real intra mode
  intra_16x16_luma_mode_t intra_luma_16x16_mode;
  intra_8x8_chroma_mode_t intra_chroma_8x8_mode;

  // --> inter data
  inter_partition_mode_t inter_partition_mode[16]; // max number of partition per MB is 16 block_4x4
  MV_XY_t inter_MV[16]; // motion vector array. when encoding inter_MV use MV prediction, when decoding the real MV are stored in this array
  uint32_t nb_partition; // number of partitions [1:16]

  int16_t*  data;         // macroblock's data
} video_macroblock_t;

#endif // _VIDEO_MACROBLOCK_H_
