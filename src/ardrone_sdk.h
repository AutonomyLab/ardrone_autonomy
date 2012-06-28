#ifndef _ARDRONE_SDK_H_
#define _ARDRONE_SDK_H_

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <sys/time.h>
#include <time.h>

extern "C" {
#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Stages/vp_stages_io_file.h>
#ifdef USE_ELINUX
#include <VP_Stages/vp_stages_V4L2_i_camif.h>
#else
#include <VP_Stages/vp_stages_i_camif.h>
#endif

#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Stages/vp_stages_yuv2rgb.h>
#include <VP_Stages/vp_stages_buffer_to_picture.h>

#include <config.h>

#ifdef JPEG_CAPTURE
#include <VP_Stages/vp_stages_io_jpeg.h>
#else
#include <VLIB/Stages/vlib_stage_decode.h>
#endif
#include <video_encapsulation.h>

#include <ardrone_tool/ardrone_version.h>
#include <ardrone_tool/ardrone_tool_configuration.h>  
#include <ardrone_tool/Com/config_com.h>
#include <ardrone_tool/UI/ardrone_input.h>
#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_tool/Control/ardrone_control.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
}

extern navdata_vision_detect_t navdata_detect;
extern navdata_phys_measures_t navdata_phys;
extern navdata_demo_t navdata;
extern navdata_time_t arnavtime;

#endif
