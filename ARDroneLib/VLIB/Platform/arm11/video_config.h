#ifndef _VIDEO_CONFIG_ARM11_H_
#define _VIDEO_CONFIG_ARM11_H_

#if TARGET_CPU_ARM == 1

/* Default configuration for ARM11 (like iphone) platform */
#define DEFAULT_QUANTIZATION          (6)

#define MAX_NUM_MACRO_BLOCKS_PER_CALL (1)

#define DEFAULT_INTERNAL_STREAM_SIZE  (1024 * 8 * 2)

#define VLIB_ALLOC_ALIGN              (16)

#endif // TARGET_CPU_ARM

#endif // _VIDEO_CONFIG_ARM11_H_

