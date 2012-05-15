#ifndef _P263_HUFFMAN_H_
#define _P263_HUFFMAN_H_

#include <VLIB/video_huffman.h>


/// Macroblock type & Coded Block Pattern for Chrominance (MCBPC)

#define VLC_MCBPC_IPICTURES_NUM         9
#define VLC_MCBPC_IPICTURES_MAX_LENGTH  9

#define CBPC_CB(mcbpc) (((mcbpc).cbpc) & 1)
#define CBPC_CR(mcbpc) (((mcbpc).cbpc) & 2)

extern huffman_code_t   vlc_mcbpc_ipictures[VLC_MCBPC_IPICTURES_NUM];
extern huffman_tree_t*  vlc_mcbpc_ipictures_tree;

typedef struct _p263_mcbpc_t {
  int32_t mb_type;
  int32_t cbpc;
} p263_mcbpc_t;

extern p263_mcbpc_t mcbpc_ipictures[VLC_MCBPC_IPICTURES_NUM];


/// Coded Block Pattern for luminance (CBPY)

#define VLC_CBPY_STANDARD_NUM         16
#define VLC_CBPY_STANDARD_MAX_LENGTH  6

extern huffman_code_t   vlc_cbpy_standard[VLC_CBPY_STANDARD_NUM];
extern huffman_tree_t*  vlc_cbpy_standard_tree;

#define CBPY_MAKE( i12, i34 ) (((i34) << 2) | (i12))

#define CBPY_INTRA_Y0(cpby) ((((cbpy).intra) >> 0) & 1)
#define CBPY_INTRA_Y1(cpby) ((((cbpy).intra) >> 1) & 1)
#define CBPY_INTRA_Y2(cpby) ((((cbpy).intra) >> 2) & 1)
#define CBPY_INTRA_Y3(cpby) ((((cbpy).intra) >> 3) & 1)

#define CBPY_INTER_Y0(cpby) ((((cbpy).inter) >> 0) & 1)
#define CBPY_INTER_Y1(cpby) ((((cbpy).inter) >> 1) & 1)
#define CBPY_INTER_Y2(cpby) ((((cbpy).inter) >> 2) & 1)
#define CBPY_INTER_Y3(cpby) ((((cbpy).inter) >> 3) & 1)

typedef struct _p263_cbpy_t {
  int32_t intra;
  int32_t inter;
} p263_cbpy_t;

extern p263_cbpy_t cbpy_standard[VLC_CBPY_STANDARD_NUM - 1];


/// Transform Coefficient (TCOEF)

#define VLC_TCOEFF_NUM          103
#define VLC_TCOEFF_MAX_LENGTH   12
#define VLC_TCOEFF_ESCAPE       2

extern huffman_code_t   vlc_tcoeff[VLC_TCOEFF_NUM];
extern huffman_tree_t*  vlc_tcoeff_tree;

typedef struct _p263_tcoeff_t {
  int32_t run;
  int32_t level;
  int32_t last;
} p263_tcoeff_t;

extern p263_tcoeff_t tcoeff[VLC_TCOEFF_NUM];


/// VLC table for Motion Vector Data (MVD)

#define MVD_VLC_NUM             64
#define MVD_VLC_MAX_LENGTH      13

extern huffman_code_t   mvd_vlc[MVD_VLC_NUM];
extern huffman_tree_t*  mvd_vlc_tree;

typedef struct _p263_mvd_t {
  int32_t vector;
  int32_t differences;
} p263_mvd_t;

extern p263_mvd_t mvd[MVD_VLC_NUM];

#endif // _P263_HUFFMAN_H_
