/**
Software License Agreement (BSD)

\file      ardrone_sdk.h
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
#ifndef ARDRONE_AUTONOMY_ARDRONE_SDK_H
#define ARDRONE_AUTONOMY_ARDRONE_SDK_H

// TODO(mani-monaj): Move these two defines to CMake
#ifndef FFMPEG_SUPPORT
#define FFMPEG_SUPPORT
#endif

#ifndef USE_LINUX
#define USE_LINUX
#endif

// TODO(mani-monaj): Research more on this issue, move the flag to CMake
// The FFMPEG library INT macros fix
#if defined __cplusplus
#    define __STDC_CONSTANT_MACROS
#endif

#include <stdint.h>

#ifndef UINT64_C
#define UINT64_C(c) (c ## ULL)
#endif

extern "C"
{
#include <config.h>

#include <utils/ardrone_gen_ids.h>
#include <ardrone_tool/ardrone_version.h>
#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/ardrone_tool_configuration.h>
#include <ardrone_tool/Com/config_com.h>
#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_tool/Control/ardrone_control.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>

#include <ardrone_tool/Video/video_stage.h>
#include <ardrone_tool/Video/video_recorder_pipeline.h>
#include <ardrone_tool/Video/video_stage_latency_estimation.h>

  extern video_decoder_config_t vec;
}

#include <ardrone_autonomy/ardrone_driver.h>

#define NB_DRIVER_POST_STAGES   10

extern ARDroneDriver *ros_driver;

extern const navdata_unpacked_t* shared_raw_navdata_ptr;
extern ros::Time shared_navdata_receive_time;

extern vp_os_mutex_t navdata_lock;
extern vp_os_mutex_t video_lock;
extern vp_os_mutex_t twist_lock;

extern int32_t looprate;
extern bool realtime_navdata;
extern bool realtime_video;

extern int32_t should_exit;

#endif  // ARDRONE_AUTONOMY_ARDRONE_SDK_H
