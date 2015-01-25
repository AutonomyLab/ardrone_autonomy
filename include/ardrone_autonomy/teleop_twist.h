/**
Software License Agreement (BSD)

\file      teleop_twist.h
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
#ifndef ARDRONE_AUTONOMY_TELEOP_TWIST_H
#define ARDRONE_AUTONOMY_TELEOP_TWIST_H

#include <ardrone_autonomy/ardrone_sdk.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/LedAnim.h>
#include <ardrone_autonomy/FlightAnim.h>
#include <ardrone_autonomy/RecordEnable.h>

#define _EPS 1.0e-6

extern input_device_t teleop;

void CmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
void LandCallback(const std_msgs::Empty &msg);
void ResetCallback(const std_msgs::Empty &msg);
void TakeoffCallback(const std_msgs::Empty &msg);

// void toggleCamCallback(const std_msgs::Empty &msg);
bool SetCamChannelCallback(
    ardrone_autonomy::CamSelect::Request& request,
    ardrone_autonomy::CamSelect::Response& response);

bool ToggleCamCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response);

bool SetLedAnimationCallback(
    ardrone_autonomy::LedAnim::Request& request,
    ardrone_autonomy::LedAnim::Response& response);

bool FlatTrimCallback(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response);

bool SetFlightAnimationCallback(
    ardrone_autonomy::FlightAnim::Request& request,
    ardrone_autonomy::FlightAnim::Response& response);

bool SetRecordCallback(
    ardrone_autonomy::RecordEnable::Request &request,
    ardrone_autonomy::RecordEnable::Response& response);

// All global drone configs that should be sent on init

#define DEFAULT_CAM_STATE 0
#define DEFAULT_NAVDATA_DEMO 0

extern int cam_state;
extern int set_navdata_demo_value;
extern int32_t detect_enemy_color;
extern int32_t detect_groundstripes_color;
extern int32_t detect_indoor_hull;  // 1: Indoor Hull
extern int32_t detect_dtype;
extern int32_t detect_hori_type;
extern int32_t detect_vert_type;
extern int32_t detect_disable_placeholder;
extern int32_t detect_enable_placeholder;

#endif  // ARDRONE_AUTONOMY_TELEOP_TWIST_H
