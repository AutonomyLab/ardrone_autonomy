#ifndef _VIDEO_CONFIG_ARM9_H_
#define _VIDEO_CONFIG_ARM9_H_

# define CACHE_DLINESIZE (32)

/* Default configuration for P5+ platform */

#define VIDEO_DCT_INTERRUPT_ENABLE    (0)

#define VIDEO_DCT_USE_INTRAM          (0)

#define DEFAULT_QUANTIZATION          (6)

/*
  Help on finding a good MAX_NUM_MACRO_BLOCKS_PER_CALL on P5P
  Maximum is 10 macroblocks because P5P DCT can only handle 64 blocks at once
  This parameter is important and has to be adapted to your resolution
  (CIF, VGA,... ) in order to minimize waiting time between two calls.
  For example case like a call with a lot of blocks followed by a call
  with a small number of blocks need to be avoided.
  Ex: In QVGA a value of 6 is not optimal because there's 20 macroblocks per GOB
  so there will be 3 calls with 6 macroblocks and a call with 2 macroblocks
  (you can check with RTMON to monitor the waiting time represendted by event VIDEO_VLIB_DCT_WAIT_UEVENT)
  In the QVGA case this is a better choice to choose a value of 7 to have two calls for 7 macroblocks and
  a call for 6 macroblocks (of course 10 seems to be the better value - 2 calls - from an hardware DCT point of view).
*/
#define MAX_NUM_MACRO_BLOCKS_PER_CALL (6)

#define DEFAULT_INTERNAL_STREAM_SIZE  (1024 * 8)

#define VLIB_ALLOC_ALIGN              CACHE_DLINESIZE

#endif // _VIDEO_CONFIG_ARM9_H_
