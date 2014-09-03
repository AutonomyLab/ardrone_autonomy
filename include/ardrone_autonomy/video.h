#ifndef _VIDEO_H_
#define _VIDEO_H_

#include <ardrone_autonomy/ardrone_sdk.h>
#include <ardrone_autonomy/ardrone_driver.h>

// The maximum memory allocation
#define MAX_STREAM_WIDTH 640
#define MAX_STREAM_HEIGHT 360

/** Drone 1 */

// Drone 1 Static Stream Size & PIP Stuff
// PIP is not supported in AR-Drone SDK 2.0
#define D1_STREAM_WIDTH 320
#define D1_STREAM_HEIGHT 240

//Vertical Camera standalone
#define D1_VERTSTREAM_WIDTH 174
#define D1_VERTSTREAM_HEIGHT 144

// Vertical Camera in PIP
#define D1_MODE2_PIP_WIDTH 87 //Huh?
#define D1_MODE2_PIP_HEIGHT 72

//Horizontal Camera in PIP
#define D1_MODE3_PIP_WIDTH 58
#define D1_MODE3_PIP_HEIGHT 42

/** Drone 2 */

// NO PIP, Both camera streams provide the same reseloution: Simple!
#define D2_STREAM_WIDTH 640
#define D2_STREAM_HEIGHT 360

extern video_com_multisocket_config_t icc;
extern const vp_api_stage_funcs_t vp_stages_export_funcs;
extern unsigned char buffer[]; // size STREAM_WIDTH * STREAM_HEIGHT * 3
extern long int current_frame_id; // this will be incremented for every frame
extern long int current_navdata_id;

#endif

