/*
 *  video_stage.h
 *  Test
 *
 *  Created by Frédéric D'HAEYER on 22/02/10.
 *  Copyright 2010 Parrot SA. All rights reserved.
 *
 */
#ifndef _VIDEO_STAGE_H_
#define _VIDEO_STAGE_H_

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_picture.h>

#include <VP_Os/vp_os.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_signal.h>
#include <VP_Os/vp_os_malloc.h>

#include <VLIB/Stages/vlib_stage_decode.h>
#include <VP_Stages/vp_stages_yuv2rgb.h>
#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Com/config_com.h>
#include <ardrone_tool/Video/video_com_stage.h>
//#define RECORD_VIDEO
//#ifdef RECORD_VIDEO
//#    include <ardrone_tool/Video/video_stage_recorder.h>
//#endif

PROTO_THREAD_ROUTINE(video_stage, data);

typedef struct _video_stage_config_t
{
	vp_os_mutex_t mutex;
	uint32_t widthImage;
	uint32_t heightImage;
	uint32_t bytesPerPixel;
	uint32_t num_picture_decoded; // Number of pictures decoded
	uint32_t num_frame; // Frame index of the last decoded picture, for association with navdata detect_tag frame index
	void* data;
} video_stage_config_t;

static C_RESULT video_stage_open(vlib_stage_decoding_config_t *cfg);
static C_RESULT video_stage_transform(vlib_stage_decoding_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out);
static C_RESULT video_stage_close(vlib_stage_decoding_config_t *cfg);

void video_stage_init(void);
void video_stage_suspend_thread(void);
void video_stage_resume_thread(void);
uint32_t video_stage_get_num_retries(void);
video_stage_config_t* video_stage_get(void);

#endif // _VIDEO_STAGE_H_
