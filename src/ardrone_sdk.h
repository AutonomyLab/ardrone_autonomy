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
//#include <VP_Api/vp_api.h>
//#include <VP_Api/vp_api_error.h>
//#include <VP_Api/vp_api_stage.h>
//#include <VP_Api/vp_api_picture.h>
//#include <VP_Stages/vp_stages_io_file.h>
//#include <VP_Stages/vp_stages_i_camif.h>
//
//#include <VP_Os/vp_os_print.h>
//#include <VP_Os/vp_os_malloc.h>
//#include <VP_Os/vp_os_delay.h>
//#include <VP_Stages/vp_stages_yuv2rgb.h>
//#include <VP_Stages/vp_stages_buffer_to_picture.h>
//#include <VLIB/Stages/vlib_stage_decode.h>
    
   

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

extern video_decoder_config_t vec;
}

#define NB_DRIVER_POST_STAGES   10

extern navdata_vision_detect_t navdata_detect;
extern navdata_phys_measures_t navdata_phys;
extern navdata_demo_t navdata;
extern navdata_time_t arnavtime;


#endif
