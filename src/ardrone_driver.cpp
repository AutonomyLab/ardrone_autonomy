#include "ardrone_driver.h"
#include "teleop_twist.h"
#include "video.h"
#include "ardrone_autonomy/LedAnim.h"
#include <signal.h>

////////////////////////////////////////////////////////////////////////////////
// class ARDroneDriver
////////////////////////////////////////////////////////////////////////////////

ARDroneDriver::ARDroneDriver()
	: image_transport(node_handle)
{
    inited = false;
	cmd_vel_sub = node_handle.subscribe("cmd_vel", 100, &cmdVelCallback);
	takeoff_sub = node_handle.subscribe("ardrone/takeoff", 1, &takeoffCallback);
	reset_sub = node_handle.subscribe("ardrone/reset", 1, &resetCallback);
	land_sub = node_handle.subscribe("ardrone/land", 1, &landCallback);
	image_pub = image_transport.advertiseCamera("ardrone/image_raw", 10);
    hori_pub = image_transport.advertiseCamera("ardrone/front/image_raw", 10);
	vert_pub = image_transport.advertiseCamera("ardrone/bottom/image_raw", 10);
    navdata_pub = node_handle.advertise<ardrone_autonomy::Navdata>("ardrone/navdata", 25);
    imu_pub = node_handle.advertise<sensor_msgs::Imu>("ardrone/imu", 25);
	toggleCam_service = node_handle.advertiseService("ardrone/togglecam", toggleCamCallback);
	toggleNavdataDemo_service = node_handle.advertiseService("ardrone/togglenavdatademo", toggleNavdataDemoCallback);
	setCamChannel_service = node_handle.advertiseService("ardrone/setcamchannel",setCamChannelCallback );
	setLedAnimation_service = node_handle.advertiseService("ardrone/setledanimation", setLedAnimationCallback);
//	setEnemyColor_service = node_handle.advertiseService("/ardrone/setenemycolor", setEnemyColorCallback);
//	setHullType_service = node_handle.advertiseService("/ardrone/sethulltype", setHullTypeCallback);

    droneFrameId = (ros::param::get("~drone_frame_id", droneFrameId)) ? droneFrameId : "ardrone_frame";
    droneFrameBase = droneFrameId + "_base";
    droneFrameIMU = droneFrameId + "_imu";
    droneFrameFrontCam = droneFrameId + "_frontcam";
    droneFrameBottomCam = droneFrameId + "_bottomcam";

    // Fill constant parts of IMU Message

    // No covariance yet for linear acceleration
    for (sensor_msgs::Imu::_angular_velocity_covariance_type::iterator ita = imu_msg.linear_acceleration_covariance.begin();
         ita != imu_msg.linear_acceleration_covariance.end(); ita++)
    {
        *ita = 0.0;
    }

    // No covariance yet for rotation
    for (sensor_msgs::Imu::_orientation_covariance_type::iterator ito = imu_msg.orientation_covariance.begin();
         ito != imu_msg.orientation_covariance.end(); ito++)
    {
        *ito = 0.0;
    }

    // No angular velocity at all
    imu_msg.angular_velocity_covariance[0] = -1.0;

}

ARDroneDriver::~ARDroneDriver()
{
}

void ARDroneDriver::run()
{
	ros::Rate loop_rate(30);
    ros::Time startTime = ros::Time::now();

	while (node_handle.ok())
	{
        if (!inited) // Give the Drone 5s of free time to finish init phase
        {
            if (((ros::Time::now() - startTime).toSec()) > 5.0)
            {
                inited = true;
                ROS_INFO("Successfully connected to '%s' (AR-Drone %d.0 - Firmware: %s) - Battery(\%): %d",
                         ardrone_control_config.ardrone_name,
                         (IS_ARDRONE1) ? 1 : 2,
                         ardrone_control_config.num_version_soft,
                         navdata.vbat_flying_percentage);
            }
        } else {
            if (current_frame_id != last_frame_id)
            {
                publish_video();
                publish_navdata();
                publish_tf();
                last_frame_id = current_frame_id;
            }
        }
        ros::spinOnce();
		loop_rate.sleep();
	}
    printf("ROS loop terminated ... \n");
}

double ARDroneDriver::getRosParam(char* param, double defaultVal)
{
    std::string name(param);
    double res, ret;
    ret =  (ros::param::get(name, res)) ? res : defaultVal;
    ROS_INFO("SET %-30s: %4.2f", param, ret);
    return ret;
}

void ARDroneDriver::publish_video()
{
    if (
            (image_pub.getNumSubscribers() == 0) &&
            (hori_pub.getNumSubscribers() == 0) &&
            (vert_pub.getNumSubscribers() == 0)
       ) return;

    if (IS_ARDRONE1)
    {
        /*
        * Information on buffer and image sizes.
        * Buffer is always in QVGA size, however for different Camera Modes
        * The picture and PIP sizes are different.
        * 
        * image_raw and buffer are always 320x240. In order to preserve backward compatibilty image_raw remains
        * always as before. Two new set of topics are added for two new cameras : /ardrone/front/xxx and /ardrone/bottom/xxx
        * 
        * In Camera State 0 front image relays the buffer  and image_raw and bottom image are not updated.
        * 
        * In Camera State 1 bottom image is a 174x144 crop of the buffer. The front image is not updated
        * 
        * In Camera State 2 bottom image is a PIP cut of size (87x72) from buffer.
        * The bottom image is a (320-87)x(240) cut of the buffer.
        * 
        * In Camera State 3 front image is a PIP cut of size (58x42) from buffer.
        * The bottom image is a (174-58)x144 crop of the buffer. 
        */
        sensor_msgs::Image image_msg;
        sensor_msgs::CameraInfo cinfo_msg;
        sensor_msgs::Image::_data_type::iterator _it;

        image_msg.header.stamp = ros::Time::now();
        cinfo_msg.header.stamp = ros::Time::now();
        if ((cam_state == ZAP_CHANNEL_HORI) || (cam_state == ZAP_CHANNEL_LARGE_HORI_SMALL_VERT))
        {
            image_msg.header.frame_id = droneFrameFrontCam;
            cinfo_msg.header.frame_id = droneFrameFrontCam;
        }
        else if ((cam_state == ZAP_CHANNEL_VERT) || (cam_state == ZAP_CHANNEL_LARGE_VERT_SMALL_HORI))
        {
            image_msg.header.frame_id = droneFrameBottomCam;
            cinfo_msg.header.frame_id = droneFrameBottomCam;
        }
        else
        {
            ROS_WARN_ONCE("Something is wrong with camera channel config.");
        }

        image_msg.width = D1_STREAM_WIDTH;
        image_msg.height = D1_STREAM_HEIGHT;
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = false;
        image_msg.step = D1_STREAM_WIDTH*3;
        image_msg.data.resize(D1_STREAM_WIDTH*D1_STREAM_HEIGHT*3);
        std::copy(buffer, buffer+(D1_STREAM_WIDTH*D1_STREAM_HEIGHT*3), image_msg.data.begin());

        // We only put the width and height in here.
        cinfo_msg.width = D1_STREAM_WIDTH;
        cinfo_msg.height = D1_STREAM_HEIGHT;

        image_pub.publish(image_msg, cinfo_msg);
        if (cam_state == ZAP_CHANNEL_HORI)
        {
            /*
            * Horizontal camera is activated, only /ardrone/front/ is being updated 
            */
            hori_pub.publish(image_msg, cinfo_msg);
        }
        else if (cam_state == ZAP_CHANNEL_VERT)
        {
            /*
            * Vertical camera is activated, only /ardrone/bottom/ is being updated 
            */
            image_msg.width = D1_VERTSTREAM_WIDTH;
            image_msg.height = D1_VERTSTREAM_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = D1_VERTSTREAM_WIDTH*3;
            image_msg.data.clear();
            image_msg.data.resize(D1_VERTSTREAM_WIDTH*D1_VERTSTREAM_HEIGHT*3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_VERTSTREAM_HEIGHT ; row++)
            {
                int _b = row * D1_STREAM_WIDTH * 3;
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_VERTSTREAM_WIDTH;
            cinfo_msg.height = D1_VERTSTREAM_HEIGHT;
            vert_pub.publish(image_msg, cinfo_msg);
        }
        else if (cam_state == ZAP_CHANNEL_LARGE_HORI_SMALL_VERT)
        {
            /*
            * The Picture in Picture is activated with vertical camera inside the horizontal
            * camera. Both /ardrone/front and /ardrone/bottom is being updated
            */

            // Front (Cropping the first 88 columns)
            image_msg.width = D1_STREAM_WIDTH  - D1_MODE2_PIP_WIDTH;
            image_msg.height = D1_STREAM_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = (D1_STREAM_WIDTH-D1_MODE2_PIP_WIDTH)*3;
            image_msg.data.clear();
            image_msg.data.resize((D1_STREAM_WIDTH - D1_MODE2_PIP_WIDTH)*D1_STREAM_HEIGHT*3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_STREAM_HEIGHT; row++)
            {
                int _b = (row * D1_STREAM_WIDTH * 3) + (D1_MODE2_PIP_WIDTH * 3);
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_STREAM_WIDTH - D1_MODE2_PIP_WIDTH;
            cinfo_msg.height = D1_STREAM_HEIGHT;
            hori_pub.publish(image_msg, cinfo_msg);

            //Bottom
            image_msg.width = D1_MODE2_PIP_WIDTH;
            image_msg.height = D1_MODE2_PIP_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = D1_MODE2_PIP_WIDTH * 3;
            image_msg.data.clear();
            image_msg.data.resize(D1_MODE2_PIP_WIDTH * D1_MODE2_PIP_HEIGHT * 3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_MODE2_PIP_HEIGHT; row++)
            {
                int _b = row * D1_STREAM_WIDTH * 3;
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_MODE2_PIP_WIDTH;
            cinfo_msg.height = D1_MODE2_PIP_HEIGHT;
            vert_pub.publish(image_msg, cinfo_msg);
        }
        else if (cam_state == ZAP_CHANNEL_LARGE_VERT_SMALL_HORI)
        {
            /*
            * The Picture in Picture is activated with horizontal camera inside the vertical
            * camera. Both /ardrone/front and /ardrone/bottom is being updated
            */

            // Bottom  (Cropping the first 58 columns)
            image_msg.width = D1_VERTSTREAM_WIDTH   - D1_MODE3_PIP_WIDTH;
            image_msg.height = D1_VERTSTREAM_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = (D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH)*3;
            image_msg.data.clear();
            image_msg.data.resize((D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH)* D1_VERTSTREAM_HEIGHT*3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_VERTSTREAM_HEIGHT; row++)
            {
                int _b = (row * (D1_STREAM_WIDTH * 3)) + (D1_MODE3_PIP_WIDTH * 3);
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH;
            cinfo_msg.height = D1_VERTSTREAM_HEIGHT;
            vert_pub.publish(image_msg, cinfo_msg);

            //Front
            image_msg.width = D1_MODE3_PIP_WIDTH;
            image_msg.height = D1_MODE3_PIP_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = D1_MODE3_PIP_WIDTH * 3;
            image_msg.data.clear();
            image_msg.data.resize(D1_MODE3_PIP_WIDTH * D1_MODE3_PIP_HEIGHT * 3);
            _it = image_msg.data.begin();
            for (int row = 0; row < D1_MODE3_PIP_HEIGHT; row++)
            {
                int _b = row * D1_STREAM_WIDTH * 3;
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }

            cinfo_msg.width = D1_MODE3_PIP_WIDTH;
            cinfo_msg.height = D1_MODE3_PIP_HEIGHT;
            hori_pub.publish(image_msg, cinfo_msg);
        }
    }
    
    /**
     * For Drone 2 w/ SDK2. Both camera streams are 360p.
     * No 720p support for now.
     * SDK 2.0 Does not support PIP.
     */
    if (IS_ARDRONE2)
    {
        sensor_msgs::Image image_msg;
        sensor_msgs::CameraInfo cinfo_msg;
        sensor_msgs::Image::_data_type::iterator _it;

        image_msg.header.stamp = ros::Time::now();
        cinfo_msg.header.stamp = ros::Time::now();
        if (cam_state == ZAP_CHANNEL_HORI)
        {
            image_msg.header.frame_id = droneFrameFrontCam;
            cinfo_msg.header.frame_id = droneFrameFrontCam;
        }
        else if (cam_state == ZAP_CHANNEL_VERT)
        {
            image_msg.header.frame_id = droneFrameBottomCam;
            cinfo_msg.header.frame_id = droneFrameBottomCam;
        }
        else
        {
            ROS_WARN_ONCE("Something is wrong with camera channel config.");
        }

        image_msg.width = D2_STREAM_WIDTH;
        image_msg.height = D2_STREAM_HEIGHT;
        image_msg.encoding = "rgb8";
        image_msg.is_bigendian = false;
        image_msg.step = D2_STREAM_WIDTH*3;
        image_msg.data.resize(D2_STREAM_WIDTH*D2_STREAM_HEIGHT*3);
        std::copy(buffer, buffer+(D2_STREAM_WIDTH*D2_STREAM_HEIGHT*3), image_msg.data.begin());

        // We only put the width and height in here.
        cinfo_msg.width = D2_STREAM_WIDTH;
        cinfo_msg.height = D2_STREAM_HEIGHT;
        image_pub.publish(image_msg, cinfo_msg); // /ardrone
        if (cam_state == ZAP_CHANNEL_HORI)
        {
            /*
            * Horizontal camera is activated, only /ardrone/front/ is being updated 
            */
            hori_pub.publish(image_msg, cinfo_msg);
        }
        else if (cam_state == ZAP_CHANNEL_VERT)
        {
            /*
            * Vertical camera is activated, only /ardrone/bottom/ is being updated 
            */
            vert_pub.publish(image_msg, cinfo_msg);
        }
    }
	
}

void ARDroneDriver::publish_navdata()
{
    if ((navdata_pub.getNumSubscribers() == 0) && (imu_pub.getNumSubscribers() == 0))
        return; // why bother, no one is listening.
	ardrone_autonomy::Navdata msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = droneFrameBase;
	msg.batteryPercent = navdata.vbat_flying_percentage;
    msg.state = (navdata.ctrl_state >> 16);
    
	// positive means counterclockwise rotation around axis
	msg.rotX = navdata.phi / 1000.0; // tilt left/right
	msg.rotY = -navdata.theta / 1000.0; // tilt forward/backward
	msg.rotZ = -navdata.psi / 1000.0; // orientation

	msg.altd = navdata.altitude; // cm
	msg.vx = navdata.vx; // mm/sec
	msg.vy = -navdata.vy; // mm/sec
	msg.vz = -navdata.vz; // mm/sec
	msg.tm = arnavtime.time;
	msg.ax = navdata_phys.phys_accs[ACC_X] / 1000.0; // g
	msg.ay = -navdata_phys.phys_accs[ACC_Y] / 1000.0; // g
	msg.az = -navdata_phys.phys_accs[ACC_Z] / 1000.0; // g
	
    // New stuff

    if (IS_ARDRONE2)
    {
        msg.magX = (int32_t)navdata_magneto.mx;
        msg.magY = (int32_t)navdata_magneto.my;
        msg.magZ = (int32_t)navdata_magneto.mz;

        msg.pressure = navdata_pressure.Pression_meas; // typo in the SDK!
        msg.temp = navdata_pressure.Temperature_meas;

        msg.wind_speed = navdata_wind.wind_speed;
        msg.wind_angle = navdata_wind.wind_angle;
        msg.wind_comp_angle = navdata_wind.wind_compensation_phi;
    }
    else
    {
        msg.magX = msg.magY = msg.magZ = 0;
        msg.pressure = 0.0;
        msg.temp = 0.0;
        msg.wind_speed = 0.0;
        msg.wind_angle = 0.0;
        msg.wind_comp_angle = 0.0;
    }

	// Tag Detection
	msg.tags_count = navdata_detect.nb_detected;
    for (int i = 0; i < navdata_detect.nb_detected; i++)
	{
		/*
		 * The tags_type is in raw format. In order to extract the information 
		 * macros from ardrone_api.h is needed.
		 *
		 * #define DETECTION_MAKE_TYPE(source,tag) ( ((source)<<16) | (tag) )
		 * #define DETECTION_EXTRACT_SOURCE(type)  ( ((type)>>16) & 0x0FF )
		 * #define DETECTION_EXTRACT_TAG(type)     ( (type) & 0x0FF )
		 * 
		 * Please also note that the xc, yc, width and height are in [0,1000] range
		 * and must get converted back based on image resolution.
		 */
		msg.tags_type.push_back(navdata_detect.type[i]);
		msg.tags_xc.push_back(navdata_detect.xc[i]);
		msg.tags_yc.push_back(navdata_detect.yc[i]);
		msg.tags_width.push_back(navdata_detect.width[i]);
		msg.tags_height.push_back(navdata_detect.height[i]);
		msg.tags_orientation.push_back(navdata_detect.orientation_angle[i]);
		msg.tags_distance.push_back(navdata_detect.dist[i]);
	}

    navdata_pub.publish(msg);

    /* IMU */
    imu_msg.header.frame_id = droneFrameIMU;
    imu_msg.header.stamp = ros::Time::now();

    // IMU - Linear Acc
    imu_msg.linear_acceleration.x = msg.ax * 9.8;
    imu_msg.linear_acceleration.y = msg.ay * 9.8;
    imu_msg.linear_acceleration.z = msg.az * 9.8;

    // IMU - Rotation Matrix
    imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(msg.rotX * _DEG2RAD, msg.rotY * _DEG2RAD, msg.rotZ * _DEG2RAD);

    imu_pub.publish(imu_msg);
}

void ARDroneDriver::publish_tf()
{
    // IMU To Base (Assume to be the same)
    tf_broad.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                    ros::Time::now(), droneFrameBase, droneFrameIMU
                    )
                );

    // Front Cam to Base
    // TODO: Precise values for Drone1 & Drone2
    tf_broad.sendTransform(
                tf::StampedTransform(
                    tf::Transform(
                        tf::createQuaternionFromRPY(0.0, 90.0 * _DEG2RAD, 0.0),
                        tf::Vector3(0.21, 0.0, 0.0)),
                    ros::Time::now(), droneFrameBase, droneFrameFrontCam
                    )
                );

    // Bottom Cam to Base (Bad Assumption: No translation from IMU and Base)
    // TODO: This should be different from Drone 1 & 2.
    tf_broad.sendTransform(
                tf::StampedTransform(
                    tf::Transform(
                        tf::createQuaternionFromRPY(0.0, 180 * _DEG2RAD, 0.0),
                        tf::Vector3(0.0, -0.02, 0.0)),
                    ros::Time::now(), droneFrameBase, droneFrameBottomCam
                    )
                );

}

void controlCHandler (int signal)
{
    ros::shutdown();
    should_exit = 1;    
}
////////////////////////////////////////////////////////////////////////////////
// custom_main
////////////////////////////////////////////////////////////////////////////////

//extern "C" int custom_main(int argc, char** argv)
int main(int argc, char** argv)
{        
        // We need to implement our own Signal handler instead of ROS to shutdown
        // the SDK threads correctly.

        ros::init(argc, argv, "ardrone_driver", ros::init_options::NoSigintHandler);
        
        signal (SIGABRT, &controlCHandler);
        signal (SIGTERM, &controlCHandler);
        signal (SIGINT, &controlCHandler);

        return ardrone_tool_main(argc, argv);
}

