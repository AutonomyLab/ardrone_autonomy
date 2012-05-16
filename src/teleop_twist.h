#ifndef _TELEOP_TWIST_H_
#define _TELEOP_TWIST_H_

#include "ardrone_sdk.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ardrone_brown/CamSelect.h>

#define _USING_SDK_1_7_

extern input_device_t teleop;

void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
void landCallback(const std_msgs::Empty &msg);
void resetCallback(const std_msgs::Empty &msg);
void takeoffCallback(const std_msgs::Empty &msg);

//void toggleCamCallback(const std_msgs::Empty &msg);
bool setCamChannelCallback(ardrone_brown::CamSelect::Request& request, ardrone_brown::CamSelect::Response& response);
bool toggleCamCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
bool toggleNavdataDemoCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

//All global drone configs that should be sent on init

#define DEFAULT_CAM_STATE 0
#define DEFAULT_NAVDATA_DEMO 1

#endif

