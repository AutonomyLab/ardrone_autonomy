#ifndef _ARDRONE_SDK_H_
#define _ARDRONE_SDK_H_

// TODO: Move these two defines to CMake
#ifndef FFMPEG_SUPPORT
#define FFMPEG_SUPPORT
#endif

#ifndef USE_LINUX
#define USE_LINUX
#endif

// TODO: Research more on this issue, move the flag to CMake
// The FFMPEG library INT macros fix
#if defined __cplusplus
#    define __STDC_CONSTANT_MACROS
#endif

#include <stdint.h>

#ifndef UINT64_C
#define UINT64_C(c) (c ## ULL)
#endif

extern "C" {

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

extern ARDroneDriver *rosDriver;

extern navdata_unpacked_t *shared_raw_navdata;
extern ros::Time shared_navdata_receive_time;

extern vp_os_mutex_t navdata_lock;
extern vp_os_mutex_t video_lock;
extern vp_os_mutex_t twist_lock;

extern int32_t looprate;
extern bool realtime_navdata;
extern bool realtime_video;

extern int32_t should_exit;

#endif
