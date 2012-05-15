#ifndef _VIDEO_UTILS_H_
#define _VIDEO_UTILS_H_

#include <VP_Os/vp_os_types.h>

////////////////////////////////////////////
// Configuration for ARM Platforms
////////////////////////////////////////////

#if TARGET_CPU_ARM == 1

// IPhone
#ifdef TARGET_OS_IPHONE
#include "arm11/video_utils.h"
#endif // TARGET_OS_IPHONE

// P6
#ifdef _ELINUX
#include "arm9_P6/video_utils.h"
#endif

#endif // TARGET_CPU_ARM

////////////////////////////////////////////
// Configuration for x86 Platforms
////////////////////////////////////////////

#if (TARGET_CPU_X86 == 1) || defined (_WIN32)
#include "x86/video_utils.h"
#endif // TARGET_CPU_X86

////////////////////////////////////////////

#endif // _VIDEO_UTILS_H_

