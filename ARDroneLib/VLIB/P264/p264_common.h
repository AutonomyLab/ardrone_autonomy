#ifndef _P264_COMMON_H_
#define _P264_COMMON_H_

#include <VP_Os/vp_os_types.h>

#define BLOCK_SIZE   4
#define BLOCK_SIZE2 16

// P6 h264 coder out data
// MB data

typedef struct _MB_intra_16x16_p264_t {
int16_t DC_Y[16];
int16_t AC_Y[256];
int16_t DC_U[4];
int16_t dummy_DC_U[12]; // hardware fake result
int16_t DC_V[4];
int16_t dummy_DC_V[12]; // hardware fake result
int16_t AC_U[64];
int16_t AC_V[64];
} MB_intra_16x16_p264_t;

typedef struct _MB_intra_4x4_p264_t {
  int16_t AC_Y[256];
  int16_t DC_U[4];
  int16_t dummy_DC_U[12]; // hardware fake result
  int16_t DC_V[4];
  int16_t dummy_DC_V[12]; // hardware fake result
  int16_t AC_U[64];
  int16_t AC_V[64];
} MB_intra_4x4_p264_t;

typedef MB_intra_4x4_p264_t MB_inter_p264_t;

typedef union _MB_intra_p264_t {
  MB_intra_16x16_p264_t intra_16x16;
  MB_intra_4x4_p264_t intra_4x4;
  MB_inter_p264_t inter;
}MB_p264_t;

// intra types

typedef enum _intra_type_t
{
  INTRA_4x4,
  INTRA_16x16
} intra_type_t;

typedef enum _intra_4x4_mode_t
{
  VERTICAL_4x4_MODE=0,
  HORIZONTAL_4x4_MODE,
  DC_4x4_MODE,
  DIAGONAL_DL_4x4_MODE,
  DIAGONAL_DR_4x4_MODE,
  VERTICAL_RIGHT_4x4_MODE,
  HORIZONTAL_DOWN_4x4_MODE,
  VERTICAL_LEFT_4x4_MODE,
  HORIZONTAL_UP_4x4_MODE
} intra_4x4_mode_t;

typedef enum _intra_16x16_luma_mode_t
{
  VERTICAL_16x16_LUMA_MODE=0,
  HORIZONTAL_16x16_LUMA_MODE,
  DC_16x16_LUMA_MODE,
  PLANE_16x16_LUMA_MODE
} intra_16x16_luma_mode_t;

typedef enum _intra_8x8_chroma_mode_t
{
  DC_8x8_CHROMA_MODE=0,
  HORIZONTAL_8x8_CHROMA_MODE,
  VERTICAL_8x8_CHROMA_MODE,
  PLANE_8x8_CHROMA_MODE
} intra_8x8_chroma_mode_t;

// inter types
#define NB_PARTITION 7
typedef enum _inter_partition_mode_t
{
  INTER_PART_16x16 = 0,
  INTER_PART_16x8 = 1,
  INTER_PART_8x16 = 2,
  INTER_PART_8x8 = 3,
  INTER_PART_8x4 = 4,
  INTER_PART_4x8 = 5,
  INTER_PART_4x4 = 6
} inter_partition_mode_t;

// motion vector
typedef struct _MV_XY_t {
  int8_t x;
  int8_t y;
} MV_XY_t;

#endif
