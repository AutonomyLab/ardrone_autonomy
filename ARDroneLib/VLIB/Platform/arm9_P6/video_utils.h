#ifndef _VIDEO_UTILS_ARM9_P6_H_
#define _VIDEO_UTILS_ARM9_P6_H_

#include <VP_Os/vp_os_types.h>

#if TARGET_CPU_ARM == 1

#include <intrin.h>

#include "video_utils_p6.h"
#include "video_dct_p6.h"
#include "video_quantizer_p6.h"
#include "video_packetizer_p6.h"
#include "UVLC/uvlc_p6.h"
#include "UVLC/uvlc_mb_layer_p6.h"

#endif // TARGET_CPU_ARM

#endif // _VIDEO_UTILS_ARM9_H_
