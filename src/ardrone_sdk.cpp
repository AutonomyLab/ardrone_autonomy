/**
Software License Agreement (BSD)

\file      ardrone_sdk.cpp
\authors   Mani Monajjemi <mmonajje@sfu.ca>
\copyright Copyright (c) 2012, Autonomy Lab (Simon Fraser University), All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ardrone_autonomy/ardrone_sdk.h>
#include <ardrone_autonomy/video.h>
#include <ardrone_autonomy/teleop_twist.h>
#include <stdint.h>

const navdata_unpacked_t* shared_raw_navdata_ptr;
ros::Time shared_navdata_receive_time;

vp_os_mutex_t navdata_lock;
vp_os_mutex_t video_lock;
vp_os_mutex_t twist_lock;

int32_t current_navdata_id = 0;

ARDroneDriver* ros_driver;

int32_t looprate;
bool realtime_navdata;
bool realtime_video;

int32_t should_exit;

extern "C"
{
  vp_stages_latency_estimation_config_t vlat;

  DEFINE_THREAD_ROUTINE(update_ros, data)
  {
    PRINT("Thread `update_ros` started \n ");
    ARDroneDriver* driver = reinterpret_cast<ARDroneDriver*>(data);
    driver->run();
    return (THREAD_RET) 0;
  }

  C_RESULT ardrone_tool_init_custom(void)
  {
    should_exit = 0;
    vp_os_mutex_init(&navdata_lock);
    vp_os_mutex_init(&video_lock);
    vp_os_mutex_init(&twist_lock);

    ros_driver = new ARDroneDriver();
    int _w, _h;

    if (IS_ARDRONE2)
    {
      ardrone_application_default_config.video_codec = H264_360P_CODEC;
      _w = D2_STREAM_WIDTH;
      _h = D2_STREAM_HEIGHT;
    }
    else if (IS_ARDRONE1)
    {
      ardrone_application_default_config.video_codec = UVLC_CODEC;
      _w = D1_STREAM_WIDTH;
      _h = D1_STREAM_HEIGHT;
    }
    else
    {
      printf("Something must be really wrong with the SDK!");
    }

    ros::param::param("~looprate", looprate, 50);
    ros::param::param("~realtime_navdata", realtime_navdata, false);
    ros::param::param("~realtime_video", realtime_video, false);
    if (!realtime_navdata) ROS_WARN("realtime navdata is off, odometry may be imprecise");

    // SET SOME NON-STANDARD DEFAULT VALUES FOR THE DRIVER
    // THESE CAN BE OVERWRITTEN BY ROS PARAMETERS (below)
    ardrone_application_default_config.bitrate_ctrl_mode = VBC_MODE_DISABLED;
    if (IS_ARDRONE2)
    {
      ardrone_application_default_config.max_bitrate = 4000;
    }

    ardrone_application_default_config.navdata_options = NAVDATA_OPTION_FULL_MASK;
    ardrone_application_default_config.video_channel = ZAP_CHANNEL_HORI;
    ardrone_application_default_config.control_level = (0 << CONTROL_LEVEL_COMBINED_YAW);
    ardrone_application_default_config.flying_mode = FLYING_MODE_FREE_FLIGHT;

    // Load ROS param to config snippet
    #include "ardrone_autonomy/snippet_ros_to_ardrone_config.h"
    // Now continue with the rest of the initialization

    ardrone_tool_input_add(&teleop);
    uint8_t post_stages_index = 0;

    // Alloc structs
    specific_parameters_t* params = reinterpret_cast<specific_parameters_t*>(
          vp_os_calloc(1, sizeof(specific_parameters_t)));
    specific_stages_t* driver_pre_stages = reinterpret_cast<specific_stages_t*>(
          vp_os_calloc(1, sizeof(specific_stages_t)));
    specific_stages_t* driver_post_stages = reinterpret_cast<specific_stages_t*>(
          vp_os_calloc(1, sizeof(specific_stages_t)));
    vp_api_picture_t* in_picture = reinterpret_cast<vp_api_picture_t*>(
          vp_os_calloc(1, sizeof(vp_api_picture_t)));
    vp_api_picture_t* out_picture = reinterpret_cast<vp_api_picture_t*>(
          vp_os_calloc(1, sizeof(vp_api_picture_t)));

    in_picture->width          = _w;
    in_picture->height         = _h;

    out_picture->framerate     = 20;
    out_picture->format        = PIX_FMT_RGB24;
    out_picture->width         = _w;
    out_picture->height        = _h;

    out_picture->y_buf         = reinterpret_cast<uint8_t*>(vp_os_malloc(_w * _h * 3));
    out_picture->cr_buf        = NULL;
    out_picture->cb_buf        = NULL;

    out_picture->y_line_size   = _w * 3;
    out_picture->cb_line_size  = 0;
    out_picture->cr_line_size  = 0;

    // Alloc the lists
    driver_pre_stages->stages_list  = NULL;
    driver_post_stages->stages_list = reinterpret_cast<vp_api_io_stage_t*>(
          vp_os_calloc(NB_DRIVER_POST_STAGES, sizeof(vp_api_io_stage_t)));

    // Fill the POST-stages------------------------------------------------------
    //        vp_os_memset (&vlat, 0x0, sizeof (vlat));
    //        vlat.state = (vp_stages_latency_estimation_state) 0;
    //        //vlat.last_decoded_frame_info= (void *)&vec;
    //        driver_post_stages->stages_list[post_stages_index].name    = "LatencyEst";
    //        driver_post_stages->stages_list[post_stages_index].type    = VP_API_FILTER_DECODER;
    //        driver_post_stages->stages_list[post_stages_index].cfg     = (void *)&vlat;
    //        driver_post_stages->stages_list[post_stages_index++].funcs = vp_stages_latency_estimation_funcs;

    driver_post_stages->stages_list[post_stages_index].name    = "ExtractData";
    driver_post_stages->stages_list[post_stages_index].type    = VP_API_OUTPUT_SDL;
    driver_post_stages->stages_list[post_stages_index].cfg     = NULL;
    driver_post_stages->stages_list[post_stages_index++].funcs   = vp_stages_export_funcs;

    driver_pre_stages->length  = 0;
    driver_post_stages->length = post_stages_index;

    params->in_pic = in_picture;
    params->out_pic = out_picture;
    params->pre_processing_stages_list  = driver_pre_stages;
    params->post_processing_stages_list = driver_post_stages;
    params->needSetPriority = 1;
    params->priority = 31;
    // Using the provided threaded pipeline implementation from SDK
    START_THREAD(video_stage, params);
    video_stage_init();
    if (ARDRONE_VERSION() >= 2)
    {
      START_THREAD(video_recorder, NULL);
      video_recorder_init();
      video_recorder_resume_thread();
    }
    // Threads do not start automatically
    video_stage_resume_thread();
    ardrone_tool_set_refresh_time(25);
    // rosDriver->configure_drone();
    START_THREAD(update_ros, ros_driver);
    return C_OK;
  }

  C_RESULT ardrone_tool_shutdown_custom()
  {
    PRINT("Shutting down ... \n ");
    JOIN_THREAD(update_ros);
    delete ros_driver;
    video_stage_resume_thread();
    ardrone_tool_input_remove(&teleop);
    return C_OK;
  }

  C_RESULT navdata_custom_init(void* data)
  {
    (void) data;
    return C_OK;
  }

  C_RESULT navdata_custom_process(const navdata_unpacked_t * const pnd)
  {
    vp_os_mutex_lock(&navdata_lock);
    shared_navdata_receive_time = ros::Time::now();
    shared_raw_navdata_ptr = reinterpret_cast<const navdata_unpacked_t*>(pnd);

    if (realtime_navdata)
    {
      // if we're publishing navdata at full speed, publish!
      const navdata_unpacked_t shared_raw_navdata = *shared_raw_navdata_ptr;
      ros_driver->PublishNavdataTypes(shared_raw_navdata, shared_navdata_receive_time);
      ros_driver->PublishNavdata(shared_raw_navdata, shared_navdata_receive_time);
      ros_driver->PublishOdometry(shared_raw_navdata, shared_navdata_receive_time);
    }

    current_navdata_id++;
    vp_os_mutex_unlock(&navdata_lock);
    return C_OK;
  }

  C_RESULT navdata_custom_release()
  {
    return C_OK;
  }

  bool_t ardrone_tool_exit()
  {
    return (should_exit == 1);
  }

  BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY(video_stage, 31)
  THREAD_TABLE_ENTRY(update_ros, 43)
  THREAD_TABLE_ENTRY(video_recorder, 20)
  THREAD_TABLE_ENTRY(navdata_update, 31)
//  THREAD_TABLE_ENTRY(ATcodec_Commands_Client, 43)
  THREAD_TABLE_ENTRY(ardrone_control, 31)
  END_THREAD_TABLE

  BEGIN_NAVDATA_HANDLER_TABLE
  NAVDATA_HANDLER_TABLE_ENTRY(
    navdata_custom_init,
    navdata_custom_process,
    navdata_custom_release,
    NULL)
  END_NAVDATA_HANDLER_TABLE
}
