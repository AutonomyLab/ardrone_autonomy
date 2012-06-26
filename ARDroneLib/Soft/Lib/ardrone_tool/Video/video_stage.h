/*
 *  video_stage.h
 *  Test
 *
 *  Created by Frederic D'HAEYER on 22/02/10.
 *  Copyright 2010 Parrot SA. All rights reserved.
 *
 */
#ifndef _VIDEO_STAGE_H_
#define _VIDEO_STAGE_H_

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

#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_tool/Video/video_stage_tcp.h>
#include <ardrone_tool/Video/video_stage_decoder.h>
#include <ardrone_tool/Video/video_stage_merge_slices.h>
#include <ardrone_tool/Video/video_stage_latency_estimation.h>

typedef struct _specific_stages_t_
{
    vp_api_io_stage_t * stages_list;
    uint8_t length;
} specific_stages_t;

typedef struct _specific_parameters_t_
{
    specific_stages_t * pre_processing_stages_list;
    specific_stages_t * post_processing_stages_list;
    vp_api_picture_t * in_pic;
    vp_api_picture_t * out_pic;
    int needSetPriority;
    int priority;
} specific_parameters_t;

extern video_decoder_config_t vec;

PROTO_THREAD_ROUTINE(video_stage, data);

static inline unsigned long RoundPower2(unsigned long x)
{
	int rval=512;
	// rval<<=1 Is A Prettier Way Of Writing rval*=2; 
	while(rval < x)
		rval<<=1;
	return rval;
}

void video_stage_init(void);
void video_stage_suspend_thread(void);
void video_stage_resume_thread(void);
uint32_t video_stage_get_num_retries(void);


#endif // _VIDEO_STAGE_H_
