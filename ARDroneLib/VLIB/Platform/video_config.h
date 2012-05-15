#ifndef _VIDEO_CONFIG_H_
#define _VIDEO_CONFIG_H_

#include <VP_Os/vp_os_types.h>

////////////////////////////////////////////
// Configuration for ARM Platforms
////////////////////////////////////////////
#if TARGET_CPU_ARM == 1
// IPhone
#if defined (USE_ANDROID) || defined (TARGET_OS_IPHONE)

#include "arm11/video_config.h"
#endif // TARGET_OS_IPHONE

// Parrot6
#ifdef _ELINUX
#include "arm9_P6/video_config.h"
#endif
#endif // TARGET_CPU_ARM

////////////////////////////////////////////
// Configuration for x86 Platforms
////////////////////////////////////////////

#if (TARGET_CPU_X86 == 1) || defined (_WIN32)

#include "x86/video_config.h"

#endif // TARGET_CPU_X86

////////////////////////////////////////////

#define DCT_BUFFER_SIZE ( MAX_NUM_MACRO_BLOCKS_PER_CALL * 6 * MCU_BLOCK_SIZE )

#endif // _VIDEO_CONFIG_H_


