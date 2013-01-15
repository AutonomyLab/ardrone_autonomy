#ifndef _ARDRONE_DRIVER_H_
#define _ARDRONE_DRIVER_H_

class ARDroneDriver;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_srvs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include "ardrone_sdk.h"
#include <vector>
#include <utils/ardrone_gen_ids.h>
#include <ardrone_tool/ardrone_version.h>
#include <ardrone_tool/ardrone_tool.h>

// Load auto-generated include files for full navdata
#define NAVDATA_STRUCTS_INCLUDES
#include "NavdataMessageDefinitions.h"
#undef NAVDATA_STRUCTS_INCLUDES


#define _DEG2RAD 0.01745331111
#define _RAD2DEG 57.2957184819

#define DRIVER_USERNAME "ardrone_driver"
#define DRIVER_APPNAME "ardrone_driver"
#define CAMERA_QUEUE_SIZE (10)
#define NAVDATA_QUEUE_SIZE (25)

enum ROOT_FRAME
{
    ROOT_FRAME_BASE = 0,
    ROOT_FRAME_FRONT = 1,
    ROOT_FRAME_BOTTOM = 2,
    ROOT_FRAME_NUM
};

class ARDroneDriver
{    
public:
	ARDroneDriver();
	~ARDroneDriver();

	void run();
    double getRosParam(char* param, double defaultVal);
    bool imuReCalibCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response &response);

    #define NAVDATA_STRUCTS_HEADER_PUBLIC
    #include "NavdataMessageDefinitions.h"
    #undef NAVDATA_STRUCTS_HEADER_PUBLIC

    void publish_video();
    void publish_navdata(navdata_unpacked_t &navdata_raw, const ros::Time &navdata_receive_time);

private:
    void publish_tf();
    bool readCovParams(std::string param_name, boost::array<double, 9> &cov_array);
    double calcAverage(std::vector<double> &vec);
    void resetCaliberation();   
    void configureDrone();

    ros::NodeHandle node_handle;
	ros::Subscriber cmd_vel_sub;
	ros::Subscriber takeoff_sub;
	ros::Subscriber reset_sub;
	ros::Subscriber land_sub;
	image_transport::ImageTransport image_transport;
	image_transport::CameraPublisher image_pub;
    image_transport::CameraPublisher hori_pub;
	image_transport::CameraPublisher vert_pub;

    camera_info_manager::CameraInfoManager *cinfo_hori_;
    camera_info_manager::CameraInfoManager *cinfo_vert_;

    ros::Publisher navdata_pub;
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;

    tf::TransformBroadcaster tf_broad;

	//ros::Subscriber toggleCam_sub;
	ros::ServiceServer toggleCam_service;
	ros::ServiceServer setCamChannel_service;
	ros::ServiceServer setLedAnimation_service;
    ros::ServiceServer imuReCalib_service;
    ros::ServiceServer flatTrim_service;
    ros::ServiceServer setFlightAnimation_service;
    ros::ServiceServer setRecord_service;
	
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

    long int last_frame_id;
    long int last_navdata_id;
    long int copy_current_frame_id;
    long int copy_current_navdata_id;

	int flying_state;

    bool inited;
    std::string droneFrameId;

    // Load auto-generated declarations for full navdata
    #define NAVDATA_STRUCTS_HEADER_PRIVATE
    #include "NavdataMessageDefinitions.h"
    #undef NAVDATA_STRUCTS_HEADER_PRIVATE

    /*
     * TF Frames
     * Base: Should be COM
     */
    std::string droneFrameBase, droneFrameIMU, droneFrameFrontCam, droneFrameBottomCam;
    int drone_root_frame;
    tf::StampedTransform tf_base_front, tf_base_bottom;

    // Huge part of IMU message is constant, let's fill'em once.
    sensor_msgs::Imu imu_msg;
    geometry_msgs::Vector3Stamped mag_msg;
    ardrone_autonomy::Navdata legacynavdata_msg;

    // Manual IMU caliberation
    bool do_caliberation;
    int max_num_samples;
    bool caliberated;
    double acc_bias[3];
    double gyro_bias[3];
    double vel_bias[3];
    std::vector< std::vector<double> > acc_samples;
    std::vector< std::vector<double> > gyro_samples;
    std::vector< std::vector<double> > vel_samples;

};

#endif
