/*
 *  video_stage.c
 *  Test
 *
 *  Created by Frederic D'HAEYER on 22/02/10.
 *  Copyright 2010 Parrot SA. All rights reserved.
 *
 */

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>

#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Com/config_com.h>
#include <ardrone_tool/Video/video_stage.h>
#include <ardrone_tool/Academy/academy_stage_recorder.h>
#include <ardrone_tool/Video/video_stage_encoded_recorder.h>

#include <ardrone_tool/ardrone_version.h>

#ifndef STREAM_WIDTH
#define STREAM_WIDTH 512
#endif
#ifndef STREAM_HEIGHT
#define STREAM_HEIGHT 512
#endif

#define NB_STAGES 5

extern char documents_dir[];
extern char resources_dir[];

PIPELINE_HANDLE video_pipeline_handle;
static bool_t video_stage_in_pause = TRUE;
static vp_os_cond_t video_stage_condition;
static vp_os_mutex_t video_stage_mutex;


static video_com_config_t icc_tcp;
static video_com_config_t icc_udp;
video_com_multisocket_config_t icc;
static video_com_config_t* icc_tab[2];

static video_stage_tcp_config_t tcpConf;


video_decoder_config_t vec;


void video_stage_init(void) {
    vp_os_mutex_init(&video_stage_mutex);
    vp_os_cond_init(&video_stage_condition, &video_stage_mutex);
}

void video_stage_suspend_thread(void) {
    vp_os_mutex_lock(&video_stage_mutex);
    video_stage_in_pause = TRUE;
    vp_os_mutex_unlock(&video_stage_mutex);
}

void video_stage_resume_thread(void) {
    vp_os_mutex_lock(&video_stage_mutex);
    vp_os_cond_signal(&video_stage_condition);
    video_stage_in_pause = FALSE;
    vp_os_mutex_unlock(&video_stage_mutex);
}

DEFINE_THREAD_ROUTINE(video_stage, data) {
    C_RESULT res;

    vp_api_io_pipeline_t pipeline;
    vp_api_io_data_t out;
    
    vp_api_io_stage_t * stages;
    video_stage_merge_slices_config_t merge_slices_cfg;
    
    uint8_t i;
    
    specific_parameters_t * params = (specific_parameters_t *)(data);

    if (1 == params->needSetPriority)
    {
      CHANGE_THREAD_PRIO (video_stage, params->priority);
    }
    
    vp_os_memset(&icc_tcp, 0, sizeof ( icc_tcp));
    vp_os_memset(&icc_udp, 0, sizeof ( icc_udp));
    
    // Video Communication config
    icc_tcp.com = COM_VIDEO();
    icc_tcp.buffer_size = (1024*1024);
    icc_tcp.protocol = VP_COM_TCP;
    COM_CONFIG_SOCKET_VIDEO(&icc_tcp.socket, VP_COM_CLIENT, VIDEO_PORT, wifi_ardrone_ip);
    
    // Video Communication config
    icc_udp.com = COM_VIDEO();
    icc_udp.buffer_size = (1024*1024);
    icc_udp.protocol = VP_COM_UDP;
    COM_CONFIG_SOCKET_VIDEO(&icc_udp.socket, VP_COM_CLIENT, VIDEO_PORT, wifi_ardrone_ip);

    icc.nb_sockets = 2;
    icc.configs = icc_tab;
    icc.forceNonBlocking = &(tcpConf.tcpStageHasMoreData);
    icc_tab[1]  = &icc_tcp;
    icc_tab[0]  = &icc_udp;
    
    icc.buffer_size = (1024*1024);
    
     vp_os_memset(&vec, 0, sizeof ( vec));

     stages = (vp_api_io_stage_t*) (vp_os_calloc(
        NB_STAGES + params->pre_processing_stages_list->length + params->post_processing_stages_list->length,
        sizeof (vp_api_io_stage_t)
    ));
    
    vec.src_picture = params->in_pic;
    vec.dst_picture = params->out_pic;
    
    vp_os_memset(&tcpConf, 0, sizeof ( tcpConf));
    tcpConf.maxPFramesPerIFrame = 30;
    tcpConf.frameMeanSize = 160*1024;
    tcpConf.tcpStageHasMoreData = FALSE;
    tcpConf.latencyDrop = 1;

    pipeline.nb_stages = 0;
    pipeline.stages = &stages[0];

    //ENCODED FRAME PROCESSING STAGES
    stages[pipeline.nb_stages].type    = VP_API_INPUT_SOCKET;
    stages[pipeline.nb_stages].cfg     = (void *) &icc;
    stages[pipeline.nb_stages++].funcs = video_com_multisocket_funcs;

    stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
    stages[pipeline.nb_stages].cfg     = (void *) &tcpConf;
    stages[pipeline.nb_stages++].funcs = video_stage_tcp_funcs;
    
    // Record Encoded video
    if(1 == ARDRONE_VERSION())
    {
        ardrone_academy_stage_recorder_config.dest.pipeline = video_pipeline_handle;
        ardrone_academy_stage_recorder_config.dest.stage    = pipeline.nb_stages;
        stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
        stages[pipeline.nb_stages].cfg     = (void*)&ardrone_academy_stage_recorder_config;
        stages[pipeline.nb_stages++].funcs = ardrone_academy_stage_recorder_funcs;
    }
    else
    {
      // Nothing to do for AR.Drone 2 as we have a separated thread for recording
    }

    //PRE-DECODING STAGES ==> recording, ...
    for(i=0;i<params->pre_processing_stages_list->length;i++){
        stages[pipeline.nb_stages].type    = params->pre_processing_stages_list->stages_list[i].type;
        stages[pipeline.nb_stages].cfg     = params->pre_processing_stages_list->stages_list[i].cfg;
        stages[pipeline.nb_stages++].funcs = params->pre_processing_stages_list->stages_list[i].funcs;
    }

    stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
    stages[pipeline.nb_stages].cfg     = (void *)&merge_slices_cfg;
    stages[pipeline.nb_stages++].funcs = video_stage_merge_slices_funcs;

    //DECODING STAGES
    stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
    stages[pipeline.nb_stages].cfg     = (void*) &vec;
    stages[pipeline.nb_stages++].funcs = video_decoding_funcs;

    //POST-DECODING STAGES ==> transformation, display, ...
    for(i=0;i<params->post_processing_stages_list->length;i++){
        stages[pipeline.nb_stages].type    = params->post_processing_stages_list->stages_list[i].type;
        stages[pipeline.nb_stages].cfg     = params->post_processing_stages_list->stages_list[i].cfg;
        stages[pipeline.nb_stages++].funcs = params->post_processing_stages_list->stages_list[i].funcs;
    }


    if (!ardrone_tool_exit()) {
        PRINT("\nvideo stage thread initialisation\n\n");

        res = vp_api_open(&pipeline, &video_pipeline_handle);

        if (SUCCEED(res)) {
            int loop = SUCCESS;
            out.status = VP_API_STATUS_PROCESSING;

            while (!ardrone_tool_exit() && (loop == SUCCESS)) {
                if (video_stage_in_pause) {
                    vp_os_mutex_lock(&video_stage_mutex);
                    icc.num_retries = VIDEO_MAX_RETRIES;
                    vp_os_cond_wait(&video_stage_condition);
                    vp_os_mutex_unlock(&video_stage_mutex);
                }

                if (SUCCEED(vp_api_run(&pipeline, &out))) {
                    if ((out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING)) {
                        loop = SUCCESS;
                    }
                } else loop = -1; // Finish this thread
            }
            
            vp_os_free(params->pre_processing_stages_list->stages_list);
            params->pre_processing_stages_list->stages_list = NULL;
            vp_os_free(params->post_processing_stages_list->stages_list);
            params->post_processing_stages_list->stages_list = NULL;
            vp_os_free(params->pre_processing_stages_list);
            params->pre_processing_stages_list = NULL;
            vp_os_free(params->post_processing_stages_list);
            params->post_processing_stages_list = NULL;
            
            vp_os_free(params->in_pic);
            params->in_pic = NULL;
            vp_os_free(params->out_pic);
            params->out_pic = NULL;
            
            vp_os_free(params);
            params = NULL;
            
            vp_api_close(&pipeline, &video_pipeline_handle);
        }
    }

    PRINT("\nvideo stage thread ended\n\n");

    return (THREAD_RET) 0;
}

uint32_t video_stage_get_num_retries(void) {
    return icc.num_retries;
}
