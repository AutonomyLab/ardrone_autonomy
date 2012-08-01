#ifndef _ARDRONE_DRIVER_H_
#define _ARDRONE_DRIVER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/Navdata2.h>
#include "ardrone_sdk.h"

class ARDroneDriver
{
public:
	ARDroneDriver();
	~ARDroneDriver();

	void run();
    double getRosParam(char* param, double defaultVal);
private:
	void publish_video();
	void publish_navdata();
	void publish_navdata2();

	ros::NodeHandle node_handle;
	ros::Subscriber cmd_vel_sub;
	ros::Subscriber takeoff_sub;
	ros::Subscriber reset_sub;
	ros::Subscriber land_sub;
	image_transport::ImageTransport image_transport;
	image_transport::CameraPublisher image_pub;
    image_transport::CameraPublisher hori_pub;
	image_transport::CameraPublisher vert_pub;

	ros::Publisher navdata_pub;
	ros::Publisher navdata2_pub;

	//ros::Subscriber toggleCam_sub;
	ros::ServiceServer toggleCam_service;
	ros::ServiceServer toggleNavdataDemo_service;
	ros::ServiceServer setCamChannel_service;
	ros::ServiceServer setLedAnimation_service;
	
	/*
	 * Orange Green : 1
	 * Orange Yellow: 2
	 * Orange Blue: 3
	 */
	//ros::ServiceServer setEnemyColor_service; 
	
	/*
	 * Indoor: 1
	 * Oudoor: 0
	 */
	//ros::ServiceServer setHullType_service;

	int last_frame_id;
	int flying_state;

};

#endif
