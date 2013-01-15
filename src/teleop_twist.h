#ifndef _TELEOP_TWIST_H_
#define _TELEOP_TWIST_H_

#include "ardrone_sdk.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/LedAnim.h>
#include <ardrone_autonomy/FlightAnim.h>
#include <ardrone_autonomy/RecordEnable.h>

#define _EPS 1.0e-6 

extern input_device_t teleop;

void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
void landCallback(const std_msgs::Empty &msg);
void resetCallback(const std_msgs::Empty &msg);
void takeoffCallback(const std_msgs::Empty &msg);

//void toggleCamCallback(const std_msgs::Empty &msg);
bool setCamChannelCallback(ardrone_autonomy::CamSelect::Request& request, ardrone_autonomy::CamSelect::Response& response);
bool toggleCamCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool setLedAnimationCallback(ardrone_autonomy::LedAnim::Request& request, ardrone_autonomy::LedAnim::Response& response);
bool flatTrimCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool setFlightAnimationCallback(ardrone_autonomy::FlightAnim::Request& request, ardrone_autonomy::FlightAnim::Response& response);
bool setRecordCallback(ardrone_autonomy::RecordEnable::Request &request, ardrone_autonomy::RecordEnable::Response& response);

//All global drone configs that should be sent on init

#define DEFAULT_CAM_STATE 0
#define DEFAULT_NAVDATA_DEMO 0

extern int cam_state;
extern int set_navdata_demo_value;
extern int32_t detect_enemy_color;
extern int32_t detect_groundstripes_color;
extern int32_t detect_indoor_hull; //1: Indoor Hull
extern int32_t detect_dtype;
extern int32_t detect_hori_type;
extern int32_t detect_vert_type;
extern int32_t detect_disable_placeholder;
extern int32_t detect_enable_placeholder;


#endif

