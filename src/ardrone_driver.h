#ifndef _ARDRONE_DRIVER_H_
#define _ARDRONE_DRIVER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <ardrone_brown/Navdata.h>
#include "ardrone_sdk.h"

class ARDroneDriver
{
public:
	ARDroneDriver();
	~ARDroneDriver();

	void run();

private:
	void publish_video();
	void publish_navdata();

	ros::NodeHandle node_handle;
	ros::Subscriber cmd_vel_sub;
	ros::Subscriber takeoff_sub;
	ros::Subscriber reset_sub;
	ros::Subscriber land_sub;
	image_transport::ImageTransport image_transport;
	image_transport::CameraPublisher image_pub;
	ros::Publisher navdata_pub;

	//ros::Subscriber toggleCam_sub;
	ros::ServiceServer toggleCam_service;

	int last_frame_id;
	int flying_state;

};

#endif
