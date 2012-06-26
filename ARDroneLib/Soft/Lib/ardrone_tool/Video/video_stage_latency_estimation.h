/*
 * video_stage_latency_estimation.h
 *
 *  Created on: Sep 14, 2011
 *      Author: s_piskorsko
 */

#ifndef VIDEO_STAGE_LATENCY_ESTIMATION_H_
#define VIDEO_STAGE_LATENCY_ESTIMATION_H_

#include <ardrone_tool/Video/vlib_stage_decode.h>

typedef enum{
	LE_DISABLED=0,
	LE_WAITING,
	LE_START,
	LE_COLOR1,
	LE_COLOR2,
}vp_stages_latency_estimation_state;



typedef struct {
	vp_stages_latency_estimation_state state;
	vp_stages_latency_estimation_state previous_state;
	int w,h;
	vlib_stage_decoding_config_t * last_decoded_frame_info;
}vp_stages_latency_estimation_config_t;

C_RESULT latency_estimation_stage_handle_message(void *cfg, PIPELINE_MSG msg_id, void *callback, void *param);
C_RESULT latency_estimation_stage_open( vp_stages_latency_estimation_config_t *cfg );
C_RESULT latency_estimation_stage_transform( vp_stages_latency_estimation_config_t *cfg , vp_api_io_data_t *in, vp_api_io_data_t *out);
C_RESULT latency_estimation_stage_close( vp_stages_latency_estimation_config_t *cfg );

extern const vp_api_stage_funcs_t vp_stages_latency_estimation_funcs;

#endif /* VIDEO_STAGE_LATENCY_ESTIMATION_H_ */
