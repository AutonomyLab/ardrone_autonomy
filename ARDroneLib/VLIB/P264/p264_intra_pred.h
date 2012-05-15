#include <VP_Os/vp_os_types.h>
#include "p264_common.h"


// 4x4 block luma Availability :
//         ------ ------
//        |      |      |
//        |  B   |  C   |
//        |      |      |
//  ------ ------ ------
// |      |      |
// |  A   |  E   |
// |      |      |
//  ------ ------
//
// E is the current block to be intra predicted based on block A,B,C. However depending on the location of E in the decoded frame
// A,B or C are not necessarily available (i.e. already decoded).
// Several cases are handled to make A,B,C available :
// - In DC_4x4_MODE, if A AND B are on a frame boundary => A,B are supposed to be a uniform block of 128 and becomes available.
// - In DIAGONAL_DL_4x4_MODE OR VERTICAL_LEFT_4x4_MODE, if C is not available => extend the down-right pixel of B on C. C becomes available.
// - in other cases, modes that use unavailable blocks are forbidden.


#define A_UNAVAILABLE (1<<0)
#define B_UNAVAILABLE (1<<1)
#define C_UNAVAILABLE (1<<2)

// 4x4 intra block reconstruction order in luma MB
//  -------------
// |  0  1  4  5 |
// |  2  3  6  7 |
// |  8  9 12 13 |
// | 10 11 14 15 |
//  -------------

// 4x4 block availability case in a frame (raster scan)
// case 0 : all available
// case 1 : A not available
// case 2 : B not available
// case 3 : A&B not available
// case 4 : C not available
// cases 1,2,3 affect DC_4x4_MODE
// case 4 affects DIAGONAL_DL_4x4_MODE and VERTICAL_LEFT_4x4_MODE
//  --------- ---------         ---------
// | 3 2 2 2 | 2 2 2 2 |       | 2 2 2 2 |
// | 1 4 0 4 | 0 4 0 4 |  ...  | 0 4 0 4 |
// | 1 0 0 4 | 0 0 0 4 |       | 0 0 0 4 |
// | 1 4 0 4 | 0 4 0 4 |       | 0 4 0 4 |
//  --------- ---------         ---------
// | 1 0 0 0 | 0 0 0 0 |       | 0 0 0 4 |
// | 1 4 0 4 | 0 4 0 4 |  ...  | 0 4 0 4 |
// | 1 0 0 4 | 0 0 0 4 |       | 0 0 0 4 |
// | 1 4 0 4 | 0 4 0 4 |       | 0 4 0 4 |
//  --------- ---------         ---------
// |                                     |
// |             ...                     |
// |                                     |
// |                                     |
//  --------- ---------         ---------

C_RESULT p264_intra_4x4_luma (intra_4x4_mode_t mode, uint8_t *picture, uint32_t picture_width, uint32_t x, uint32_t y, uint32_t linesize);

// luma 16x16 block and chroma 8x8 block availability in a frame (raster scan)
// the same rules apply as in 4x4 intra mode

// 16x16 - 8x8 block availability mode in a frame (raster scan)
// case 0 : all available
// case 1 : A not available
// case 2 : B not available
// case 3 : A&B not available
//  --- ---       ---
// | 3 | 2 | ... | 2 |
//  --- ---       ---
// | 1 | 0 | ... | 0 |
//  --- ---       ---
// |      ...        |
//  --- ---       ---

C_RESULT p264_intra_16x16_luma (intra_16x16_luma_mode_t mode, uint8_t *picture, uint32_t x, uint32_t y, uint32_t linesize);

C_RESULT p264_intra_8x8_chroma (intra_8x8_chroma_mode_t mode, uint8_t *picture, uint32_t x, uint32_t y, uint32_t linesize);

