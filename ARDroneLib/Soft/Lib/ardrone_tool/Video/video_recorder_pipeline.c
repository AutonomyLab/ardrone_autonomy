//
//  video_recorder_pipeline.c
//  ARDroneLib
//
//  Created by Nicolas Brulez on 14/10/11.
//  Copyright 2011 Parrot. All rights reserved.
//

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>

#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Com/config_com.h>
#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_tool/Video/video_stage_encoded_recorder.h>
#include <ardrone_tool/Video/video_recorder_pipeline.h>
#include <ardrone_tool/Video/video_stage_tcp.h>

#include <ardrone_tool/ardrone_version.h>

video_com_config_t record_icc;

static bool_t video_recorder_in_pause = TRUE;
static vp_os_cond_t video_recorder_condition;
static vp_os_mutex_t video_recorder_mutex;

static bool_t isInit = FALSE;

void video_recorder_init (void)
{
  vp_os_mutex_init (&video_recorder_mutex);
  vp_os_cond_init (&video_recorder_condition, &video_recorder_mutex);
  isInit = TRUE;
}

void video_recorder_suspend_thread (void)
{
  vp_os_mutex_lock (&video_recorder_mutex);
  video_recorder_in_pause = TRUE;
  vp_os_mutex_unlock (&video_recorder_mutex);
}

void video_recorder_resume_thread (void)
{
  vp_os_mutex_lock (&video_recorder_mutex);
  vp_os_cond_signal (&video_recorder_condition);
  video_recorder_in_pause = FALSE;
  vp_os_mutex_unlock (&video_recorder_mutex);
}

DEFINE_THREAD_ROUTINE(video_recorder, data)
{
  if (1 >= ARDRONE_VERSION ()) // Run only for ARDrone 2 and upper
    {
      return (THREAD_RET)0;
    }
  C_RESULT res = C_OK;
    
  vp_api_io_pipeline_t pipeline;
  vp_api_io_data_t out;
  vp_api_io_stage_t stages[3];
  PIPELINE_HANDLE video_recorder_pipeline_handle;
  video_recorder_thread_param_t *video_recorder_thread_param = (video_recorder_thread_param_t *)data;

  video_stage_tcp_config_t tcpConf;
    
  if(video_recorder_thread_param != NULL)
  {
      CHANGE_THREAD_PRIO(video_recorder, video_recorder_thread_param->priority);
      video_stage_encoded_recorder_config.finish_callback = video_recorder_thread_param->finish_callback; 
  }
    
  vp_os_memset (&record_icc, 0x0, sizeof (record_icc));
  record_icc.com = COM_VIDEO ();
  record_icc.buffer_size = (8*1024);
  record_icc.protocol = VP_COM_TCP;
  record_icc.forceNonBlocking = &tcpConf.tcpStageHasMoreData;
  COM_CONFIG_SOCKET_VIDEO (&record_icc.socket, VP_COM_CLIENT, VIDEO_RECORDER_PORT, wifi_ardrone_ip);
  record_icc.timeoutFunc = &video_stage_encoded_recorder_com_timeout;
  record_icc.timeoutFuncAfterSec = 10;
  
  vp_os_memset (&tcpConf, 0, sizeof (tcpConf));
  tcpConf.maxPFramesPerIFrame = 30;
  tcpConf.frameMeanSize = 160*1024;
  tcpConf.latencyDrop = 0;
    
  pipeline.nb_stages = 0;
  pipeline.stages = &stages[0];
    
  // Com
  stages[pipeline.nb_stages].type = VP_API_INPUT_SOCKET;
  stages[pipeline.nb_stages].cfg  = (void *)&record_icc;
  stages[pipeline.nb_stages++].funcs = video_com_funcs;

  // TCP
  stages[pipeline.nb_stages].type = VP_API_FILTER_DECODER;
  stages[pipeline.nb_stages].cfg  = (void *) &tcpConf;
  stages[pipeline.nb_stages++].funcs = video_stage_tcp_funcs;
    
  // Record
  stages[pipeline.nb_stages].type = VP_API_FILTER_DECODER;
  stages[pipeline.nb_stages].cfg  = (void *) &video_stage_encoded_recorder_config;
  stages[pipeline.nb_stages++].funcs = video_encoded_recorder_funcs;
    
  while (FALSE == isInit)
    {
        printf ("Waiting for init\n");
    }
    
  if (! ardrone_tool_exit ())
    {
      PRINT ("Video recorder thread initialisation\n");
      res = vp_api_open (&pipeline, &video_recorder_pipeline_handle);
        
      if (SUCCEED (res))
        {
          int loop = SUCCESS;
          out.status = VP_API_STATUS_PROCESSING;
            
          while (! ardrone_tool_exit () && (SUCCESS == loop))
            {
              if (video_recorder_in_pause)
                {
                  vp_os_mutex_lock (&video_recorder_mutex);
                  record_icc.num_retries = VIDEO_MAX_RETRIES;
                  vp_os_cond_wait (&video_recorder_condition);
                  vp_os_mutex_unlock (&video_recorder_mutex);
                }

              if (SUCCEED (vp_api_run (&pipeline, &out)))
                {
                  if ((VP_API_STATUS_PROCESSING == out.status) ||
                      (VP_API_STATUS_STILL_RUNNING == out.status))
                    {
                      loop = SUCCESS;
                    }
                  else loop = -1;
                }
              else loop = -1;
            }
          vp_api_close (&pipeline, &video_recorder_pipeline_handle);
        }
    }
  PRINT ("Video recorder thread ended\n");
    
  return (THREAD_RET)res;
}

uint32_t video_recorder_get_num_retries(void) {
    return record_icc.num_retries;
}
