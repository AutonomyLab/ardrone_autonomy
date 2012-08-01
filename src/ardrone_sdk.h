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

extern "C" {

#include <config.h>
    
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

#define NB_DRIVER_POST_STAGES   10

extern navdata_vision_detect_t navdata_detect;
extern navdata_phys_measures_t navdata_phys;
extern navdata_demo_t navdata;
extern navdata_time_t arnavtime;

extern navdata_pressure_raw_t navdata_pressure;
extern navdata_magneto_t navdata_magneto;
extern navdata_wind_speed_t navdata_wind;

extern int32_t should_exit;

#endif
