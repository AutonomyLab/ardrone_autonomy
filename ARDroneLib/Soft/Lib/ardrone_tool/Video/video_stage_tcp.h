//
//  video_stage_tcp.h
//  ARDroneLib
//
//  Created by nicolas on 15/07/11.
//  Copyright 2011 Parrot. All rights reserved.
//
#ifndef _VIDEO_STAGE_TCP_H_
#define _VIDEO_STAGE_TCP_H_
#include <VP_Api/vp_api.h>
#include <inttypes.h>

typedef struct _video_stage_tcp_config_t
{
    int maxPFramesPerIFrame;
    int frameMeanSize;
    int latencyDrop;

    int currentSize;
    
    uint8_t **bufferPointer;
    uint8_t *globalBuffer;
    uint8_t *frameBuffer;

    bool_t tcpStageHasMoreData;

} video_stage_tcp_config_t;

C_RESULT video_stage_tcp_open(video_stage_tcp_config_t *cfg);
C_RESULT video_stage_tcp_transform(video_stage_tcp_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT video_stage_tcp_close(video_stage_tcp_config_t *cfg);

extern const vp_api_stage_funcs_t video_stage_tcp_funcs;

#endif // _VIDEO_STAGE_TCP_H_
