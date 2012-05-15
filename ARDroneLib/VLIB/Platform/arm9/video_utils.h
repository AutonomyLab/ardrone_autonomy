#ifndef _VIDEO_UTILS_ARM9_H_
#define _VIDEO_UTILS_ARM9_H_

#include <VP_Os/vp_os_types.h>

#if TARGET_CPU_ARM == 1

#include <intrin.h>

#include "video_utils_p5p.h"
#include "video_dct_p5p.h"
#include "video_quantizer_p5p.h"
#include "video_packetizer_p5p.h"
#include "UVLC/uvlc_p5p.h"
#include "UVLC/uvlc_mb_layer_p5p.h"

#endif // TARGET_CPU_ARM

#endif // _VIDEO_UTILS_ARM9_H_
