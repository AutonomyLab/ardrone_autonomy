#ifndef _VIDEO_CONFIG_X86_H_
#define _VIDEO_CONFIG_X86_H_

/* Default configuration for x86 platform */

#if (TARGET_CPU_X86 == 1) || defined (_WIN32)

#define DEFAULT_QUANTIZATION          (6)

#define MAX_NUM_MACRO_BLOCKS_PER_CALL (1)

#define DEFAULT_INTERNAL_STREAM_SIZE  (1024 * 8)

#define VLIB_ALLOC_ALIGN              (16) /* Default alignement for using SIMD instruction */

#endif // TARGET_CPU_X86

#endif // _VIDEO_CONFIG_X86_H_
