#ifndef _ARDRONE_DRIVER_H_
#define _ARDRONE_DRIVER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include "ardrone_sdk.h"
#include <vector>

#define _DEG2RAD 0.01745331111
#define _RAD2DEG 57.2957184819

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

private:
	void publish_video();
	void publish_navdata();
    void publish_tf();
    bool readCovParams(std::string param_name, boost::array<double, 9> &cov_array);
    double calcAverage(std::vector<double> &vec);
    void resetCaliberation();    

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

    tf::TransformBroadcaster tf_broad;

	//ros::Subscriber toggleCam_sub;
	ros::ServiceServer toggleCam_service;
	ros::ServiceServer toggleNavdataDemo_service;
	ros::ServiceServer setCamChannel_service;
	ros::ServiceServer setLedAnimation_service;
    ros::ServiceServer imuReCalib_service;
	
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
	int flying_state;

    bool inited;
    std::string droneFrameId;

    // Navdata copy
    navdata_demo_t navdata;
    navdata_phys_measures_t navdata_phys;
    navdata_vision_detect_t navdata_detect;
    navdata_pressure_raw_t navdata_pressure;
    navdata_magneto_t navdata_magneto;
    navdata_wind_speed_t navdata_wind;
    navdata_time_t arnavtime;

    /*
     * TF Frames
     * Base: Should be COM
     */
    std::string droneFrameBase, droneFrameIMU, droneFrameFrontCam, droneFrameBottomCam;
    int drone_root_frame;
    tf::StampedTransform tf_base_front, tf_base_bottom;

    // Huge part of IMU message is constant, let's fill'em once.
    sensor_msgs::Imu imu_msg;

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
