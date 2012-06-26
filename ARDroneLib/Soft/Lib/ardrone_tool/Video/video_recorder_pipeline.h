//
//  video_recorder_pipeline.h
//  ARDroneEngine
//
//  Created by Nicolas Brulez on 14/10/11.
//  Copyright 2011 Parrot. All rights reserved.
//

#ifndef _VIDEO_RECORDER_PIPELINE_H_
#define _VIDEO_RECORDER_PIPELINE_H_

#include <VP_Os/vp_os.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_signal.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Api/vp_api_thread_helper.h>

#include <ardrone_tool/Video/video_stage_encoded_recorder.h>

typedef struct _video_recorder_thread_param_t_
{
    int32_t priority;
    video_stage_encoded_recorder_callback finish_callback;
} video_recorder_thread_param_t;

PROTO_THREAD_ROUTINE (video_recorder, data);

void video_recorder_init(void);
void video_recorder_suspend_thread(void);
void video_recorder_resume_thread(void);
uint32_t video_recorder_get_num_retries(void);

#endif
