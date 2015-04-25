/**
Software License Agreement (BSD)

\file      video.h
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
#ifndef ARDRONE_AUTONOMY_VIDEO_H
#define ARDRONE_AUTONOMY_VIDEO_H

#include <ardrone_autonomy/ardrone_sdk.h>
#include <ardrone_autonomy/ardrone_driver.h>
#include <stdint.h>

// The maximum memory allocation
#define MAX_STREAM_WIDTH 640
#define MAX_STREAM_HEIGHT 360

/** Drone 1 */

// Drone 1 Static Stream Size & PIP Stuff
// PIP is not supported in AR-Drone SDK 2.0
#define D1_STREAM_WIDTH 320
#define D1_STREAM_HEIGHT 240

// Vertical Camera standalone
#define D1_VERTSTREAM_WIDTH 174
#define D1_VERTSTREAM_HEIGHT 144

// Vertical Camera in PIP
#define D1_MODE2_PIP_WIDTH 87  // Huh?
#define D1_MODE2_PIP_HEIGHT 72

// Horizontal Camera in PIP
#define D1_MODE3_PIP_WIDTH 58
#define D1_MODE3_PIP_HEIGHT 42

/** Drone 2 */

// NO PIP, Both camera streams provide the same reseloution: Simple!
#define D2_STREAM_WIDTH 640
#define D2_STREAM_HEIGHT 360

extern video_com_multisocket_config_t icc;
extern const vp_api_stage_funcs_t vp_stages_export_funcs;
extern unsigned char buffer[];  // size STREAM_WIDTH * STREAM_HEIGHT * 3
extern int32_t current_frame_id;  // this will be incremented for every frame
extern int32_t current_navdata_id;
extern ros::Time shared_video_receive_time;

#endif  // ARDRONE_AUTONOMY_VIDEO_H
