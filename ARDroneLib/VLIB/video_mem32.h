#ifndef _VIDEO_COPY32_H_
#define _VIDEO_COPY32_H_

#include <VP_Os/vp_os_types.h>

// Reset length data at a specified dst memory location
// This is an optimized version that works with a dst aligned on a CACHELINE boundary
C_RESULT video_zeromem32( uint32_t* dst, uint32_t length );

C_RESULT video_copy32(uint32_t* dst, uint32_t* src, uint32_t nb);
C_RESULT video_copy32_swap(uint32_t* dst, uint32_t* src, uint32_t nb);

#endif // _VIDEO_COPY32_H_
