#ifndef _VIDEO_H_
#define _VIDEO_H_

#include "ardrone_sdk.h"

#define STREAM_WIDTH QVGA_WIDTH
#define STREAM_HEIGHT QVGA_HEIGHT

extern const vp_api_stage_funcs_t vp_stages_export_funcs;
extern unsigned char buffer[]; // size STREAM_WIDTH * STREAM_HEIGHT * 3
extern int current_frame_id; // this will be incremented for every frame

PROTO_THREAD_ROUTINE(video_update_thread, data);

#endif

