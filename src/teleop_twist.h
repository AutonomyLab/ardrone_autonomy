#ifndef _TELEOP_TWIST_H_
#define _TELEOP_TWIST_H_

#include "ardrone_sdk.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#define _USING_SDK_1_7_

extern input_device_t teleop;

void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
void landCallback(const std_msgs::Empty &msg);
void resetCallback(const std_msgs::Empty &msg);
void takeoffCallback(const std_msgs::Empty &msg);

//void toggleCamCallback(const std_msgs::Empty &msg);
bool toggleCamCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
#endif

