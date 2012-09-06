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
    last_frame_id = -1;
    last_navdata_id = -1;
    cmd_vel_sub = node_handle.subscribe("cmd_vel", 1, &cmdVelCallback);
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

    /*
        To be honest, I am not sure why advertising a service using class members should be this complicated!
        One day, I should learn what is exactly happenning here. /M
    */
    imuReCalib_service = node_handle.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
            ("ardrone/imu_recalib", boost::bind(&ARDroneDriver::imuReCalibCallback, this, _1, _2));

//	setEnemyColor_service = node_handle.advertiseService("/ardrone/setenemycolor", setEnemyColorCallback);
//	setHullType_service = node_handle.advertiseService("/ardrone/sethulltype", setHullTypeCallback);

    droneFrameId = (ros::param::get("~drone_frame_id", droneFrameId)) ? droneFrameId : "ardrone_base";
    droneFrameBase = droneFrameId + "_link";
    droneFrameIMU = droneFrameId + "_imu";
    droneFrameFrontCam = droneFrameId + "_frontcam";
    droneFrameBottomCam = droneFrameId + "_bottomcam";

    drone_root_frame = (ros::param::get("~root_frame", drone_root_frame)) ? drone_root_frame : ROOT_FRAME_BASE;
    drone_root_frame = drone_root_frame % ROOT_FRAME_NUM;
    ROS_INFO("Root Frame is: %d", drone_root_frame);

    // Fill constant parts of IMU Message
    // If no rosparam is set then the default value of 0.0 will be assigned to all covariance values

    for (int i = 0; i < 9; i++)
    {
        imu_msg.linear_acceleration_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.orientation_covariance[i] = 0.0;
    }
    readCovParams("~cov/imu_la", imu_msg.linear_acceleration_covariance);
    readCovParams("~cov/imu_av", imu_msg.angular_velocity_covariance);
    readCovParams("~cov/imu_or", imu_msg.orientation_covariance);

    // Caliberation
    max_num_samples = 50;
    do_caliberation = (ros::param::get("~do_imu_caliberation", do_caliberation)) ? do_caliberation : false;
    if (do_caliberation) {
        resetCaliberation();
        ROS_WARN("Automatic IMU Caliberation is active.");
    }

    // Camera Info Manager
    cinfo_hori_ = new camera_info_manager::CameraInfoManager(ros::NodeHandle("ardrone/front"), "ardrone_front");
    cinfo_vert_ = new camera_info_manager::CameraInfoManager(ros::NodeHandle("ardrone/bottom"), "ardrone_bottom");

    // TF Stuff


    // Front Cam to Base
    // TODO: Precise values for Drone1 & Drone2
    tf_base_front = tf::StampedTransform(
                tf::Transform(
                    tf::createQuaternionFromRPY(-90.0 * _DEG2RAD, 0.0, -90.0 * _DEG2RAD),
                    tf::Vector3(0.21, 0.0, 0.0)),
                ros::Time::now(), droneFrameBase, droneFrameFrontCam
                );


    // Bottom Cam to Base (Bad Assumption: No translation from IMU and Base)
    // TODO: This should be different from Drone 1 & 2.
    tf_base_bottom = tf::StampedTransform(
                tf::Transform(
                    tf::createQuaternionFromRPY(180.0 * _DEG2RAD, 0.0, 90.0 * _DEG2RAD),
                    tf::Vector3(0.0, -0.02, 0.0)),
                ros::Time::now(), droneFrameBase, droneFrameBottomCam
                );

    // Changing the root for TF if needed
    if (drone_root_frame == ROOT_FRAME_FRONT)
    {
        tf_base_front.setData(tf_base_front.inverse());
        tf_base_front.child_frame_id_.swap(tf_base_front.frame_id_);
    }
    else if (drone_root_frame == ROOT_FRAME_BOTTOM)
    {
        tf_base_bottom.setData(tf_base_bottom.inverse());
        tf_base_bottom.child_frame_id_.swap(tf_base_bottom.frame_id_);
    }   

}

ARDroneDriver::~ARDroneDriver()
{
    delete cinfo_hori_;
    delete cinfo_vert_;
}

void ARDroneDriver::run()
{
    // TODO: 50Hz made navdata unstable, I think it is a locking issue.
    ros::Rate loop_rate(50);
    ros::Time startTime = ros::Time::now();
    static int freq_dev = 0;
	while (node_handle.ok())
	{
        if (!inited) // Give the Drone 5s of free time to finish init phase
        {
            if (((ros::Time::now() - startTime).toSec()) > 5.0)
            {
                inited = true;
                vp_os_mutex_lock(&navdata_lock);
                ROS_INFO("Successfully connected to '%s' (AR-Drone %d.0 - Firmware: %s) - Battery(\%): %d",
                         ardrone_control_config.ardrone_name,
                         (IS_ARDRONE1) ? 1 : 2,
                         ardrone_control_config.num_version_soft,
                         shared_navdata.vbat_flying_percentage);
                vp_os_mutex_unlock(&navdata_lock);
                if (ardrone_control_config.num_version_soft[0] == '0')
                {
                    ROS_WARN("The AR-Drone has a suspicious Firmware number. It usually means the network link is unreliable.");
                }
            }
        } else {
            if (current_frame_id != last_frame_id)
            {
                publish_video();                                
                last_frame_id = current_frame_id;
            }
            if (current_navdata_id != last_navdata_id)
            {
                publish_navdata();
                last_navdata_id = current_navdata_id;
            }
            if (freq_dev == 0) publish_tf();

            freq_dev = (freq_dev + 1) % 5; // ~10Hz TF publish
        }
        ros::spinOnce();
		loop_rate.sleep();
	}
    printf("ROS loop terminated ... \n");
}

void ARDroneDriver::resetCaliberation()
{
    caliberated = false;
    acc_samples.clear();
    gyro_samples.clear();
    vel_samples.clear();
    for (int i = 0; i < 3; i++)
    {
        acc_bias[i] = 0.0;
        vel_bias[i] = 0.0;
        gyro_bias[i] = 0.0;
        acc_samples.push_back(std::vector<double> ());
        gyro_samples.push_back(std::vector<double> ());
        vel_samples.push_back(std::vector<double> ());
    }
}

double ARDroneDriver::calcAverage(std::vector<double> &vec)
{
    double ret = 0.0;
    for (int i = 0; i < vec.size(); i++)
    {
        ret += vec.at(i);
    }
    return (ret / vec.size());
}

bool ARDroneDriver::readCovParams(std::string param_name, boost::array<double, 9> &cov_array)
{
    XmlRpc::XmlRpcValue cov_list;
    std::stringstream str_stream;
    if (ros::param::get(param_name, cov_list))
    {
         if (cov_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
         {
             ROS_WARN("Covariance values for %s is not a list", param_name.c_str());
             return false;
         }

         if (cov_list.size() != 9)
         {
             ROS_WARN("Covariance list size for %s is not of size 9 (Size: %d)", param_name.c_str(), cov_list.size());
             return false;
         }
         str_stream << param_name << " set to [";
         for (int32_t i = 0; i < cov_list.size(); i++)
         {
             ROS_ASSERT(cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
             cov_array[i] = static_cast<double> (cov_list[i]);
             str_stream << cov_array[i] << ((i < 8) ? ", " : "");
         }
         str_stream << "]";
         ROS_INFO(str_stream.str().c_str());
         return true;

    }
    else
    {
        ROS_INFO("No values found for `%s` in parameters, set all to zero.", param_name.c_str());
        return false;
    }

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

    // Camera Info (NO PIP)

    sensor_msgs::CameraInfo cinfo_msg_hori = cinfo_hori_->getCameraInfo();
    sensor_msgs::CameraInfo cinfo_msg_vert = cinfo_vert_->getCameraInfo();

    cinfo_msg_hori.header.frame_id = droneFrameFrontCam;
    cinfo_msg_vert.header.frame_id = droneFrameBottomCam;

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
        sensor_msgs::Image::_data_type::iterator _it;

        image_msg.header.stamp = ros::Time::now();
        if ((cam_state == ZAP_CHANNEL_HORI) || (cam_state == ZAP_CHANNEL_LARGE_HORI_SMALL_VERT))
        {
            image_msg.header.frame_id = droneFrameFrontCam;
        }
        else if ((cam_state == ZAP_CHANNEL_VERT) || (cam_state == ZAP_CHANNEL_LARGE_VERT_SMALL_HORI))
        {
            image_msg.header.frame_id = droneFrameBottomCam;
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

        vp_os_mutex_lock(&video_lock);
        std::copy(buffer, buffer+(D1_STREAM_WIDTH*D1_STREAM_HEIGHT*3), image_msg.data.begin());
        vp_os_mutex_unlock(&video_lock);

        if (cam_state == ZAP_CHANNEL_HORI)
        {
            /*
            * Horizontal camera is activated, only /ardrone/front/ is being updated 
            */
            cinfo_msg_hori.width = D1_STREAM_WIDTH;
            cinfo_msg_hori.height = D1_STREAM_HEIGHT;

            image_pub.publish(image_msg, cinfo_msg_hori);
            hori_pub.publish(image_msg, cinfo_msg_hori);
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
            vp_os_mutex_lock(&video_lock);
            for (int row = 0; row < D1_VERTSTREAM_HEIGHT ; row++)
            {
                int _b = row * D1_STREAM_WIDTH * 3;
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }
            vp_os_mutex_unlock(&video_lock);

            cinfo_msg_vert.width = D1_VERTSTREAM_WIDTH;
            cinfo_msg_vert.height = D1_VERTSTREAM_HEIGHT;
            image_pub.publish(image_msg, cinfo_msg_vert);
            vert_pub.publish(image_msg, cinfo_msg_vert);
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
            vp_os_mutex_lock(&video_lock);
            for (int row = 0; row < D1_STREAM_HEIGHT; row++)
            {
                int _b = (row * D1_STREAM_WIDTH * 3) + (D1_MODE2_PIP_WIDTH * 3);
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }
            vp_os_mutex_unlock(&video_lock);

            cinfo_msg_hori.width = D1_STREAM_WIDTH - D1_MODE2_PIP_WIDTH;
            cinfo_msg_hori.height = D1_STREAM_HEIGHT;
            hori_pub.publish(image_msg, cinfo_msg_hori);

            //Bottom
            image_msg.width = D1_MODE2_PIP_WIDTH;
            image_msg.height = D1_MODE2_PIP_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = D1_MODE2_PIP_WIDTH * 3;
            image_msg.data.clear();
            image_msg.data.resize(D1_MODE2_PIP_WIDTH * D1_MODE2_PIP_HEIGHT * 3);
            _it = image_msg.data.begin();
            vp_os_mutex_lock(&video_lock);
            for (int row = 0; row < D1_MODE2_PIP_HEIGHT; row++)
            {
                int _b = row * D1_STREAM_WIDTH * 3;
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }
            vp_os_mutex_unlock(&video_lock);

            cinfo_msg_vert.width = D1_MODE2_PIP_WIDTH;
            cinfo_msg_vert.height = D1_MODE2_PIP_HEIGHT;
            vert_pub.publish(image_msg, cinfo_msg_vert);
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
                vp_os_mutex_lock(&video_lock);
                _it = std::copy(buffer + _b, buffer + _e, _it);
                vp_os_mutex_unlock(&video_lock);
            }

            cinfo_msg_vert.width = D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH;
            cinfo_msg_vert.height = D1_VERTSTREAM_HEIGHT;
            vert_pub.publish(image_msg, cinfo_msg_vert);

            //Front
            image_msg.width = D1_MODE3_PIP_WIDTH;
            image_msg.height = D1_MODE3_PIP_HEIGHT;
            image_msg.encoding = "rgb8";
            image_msg.is_bigendian = false;
            image_msg.step = D1_MODE3_PIP_WIDTH * 3;
            image_msg.data.clear();
            image_msg.data.resize(D1_MODE3_PIP_WIDTH * D1_MODE3_PIP_HEIGHT * 3);
            _it = image_msg.data.begin();
            vp_os_mutex_lock(&video_lock);
            for (int row = 0; row < D1_MODE3_PIP_HEIGHT; row++)
            {
                int _b = row * D1_STREAM_WIDTH * 3;
                int _e = _b + image_msg.step;
                _it = std::copy(buffer + _b, buffer + _e, _it);
            }vp_os_mutex_unlock(&video_lock);

            cinfo_msg_hori.width = D1_MODE3_PIP_WIDTH;
            cinfo_msg_hori.height = D1_MODE3_PIP_HEIGHT;
            hori_pub.publish(image_msg, cinfo_msg_hori);
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
        sensor_msgs::Image::_data_type::iterator _it;

        image_msg.header.stamp = ros::Time::now();
        cinfo_msg_hori.header.stamp = ros::Time::now();
        cinfo_msg_vert.header.stamp = ros::Time::now();

        if (cam_state == ZAP_CHANNEL_HORI)
        {
            image_msg.header.frame_id = droneFrameFrontCam;
        }
        else if (cam_state == ZAP_CHANNEL_VERT)
        {
            image_msg.header.frame_id = droneFrameBottomCam;
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
        vp_os_mutex_lock(&video_lock);
        std::copy(buffer, buffer+(D2_STREAM_WIDTH*D2_STREAM_HEIGHT*3), image_msg.data.begin());
        vp_os_mutex_unlock(&video_lock);
        // We only put the width and height in here.


        if (cam_state == ZAP_CHANNEL_HORI)
        {
            /*
            * Horizontal camera is activated, only /ardrone/front/ is being updated 
            */
            cinfo_msg_hori.width = D2_STREAM_WIDTH;
            cinfo_msg_hori.height = D2_STREAM_HEIGHT;
            image_pub.publish(image_msg, cinfo_msg_hori); // /ardrone
            hori_pub.publish(image_msg, cinfo_msg_hori);
        }
        else if (cam_state == ZAP_CHANNEL_VERT)
        {
            /*
            * Vertical camera is activated, only /ardrone/bottom/ is being updated 
            */
            cinfo_msg_vert.width = D2_STREAM_WIDTH;
            cinfo_msg_vert.height = D2_STREAM_HEIGHT;
            image_pub.publish(image_msg, cinfo_msg_vert); // /ardrone
            vert_pub.publish(image_msg, cinfo_msg_vert);
        }
    }
	
}

void ARDroneDriver::publish_navdata()
{
    // Thread safe copy of interesting Navdata data
    vp_os_mutex_lock(&navdata_lock);
    navdata_detect = shared_navdata_detect;
    navdata_phys = shared_navdata_phys;
    navdata = shared_navdata;
    arnavtime = shared_arnavtime;
    if (IS_ARDRONE2)
    { // This is neccessary
        navdata_pressure = shared_navdata_pressure;
        navdata_magneto = shared_navdata_magneto;
        navdata_wind = shared_navdata_wind;
    }
    vp_os_mutex_unlock(&navdata_lock);


    if ((do_caliberation) && (!caliberated))
    {
        acc_samples[0].push_back(navdata_phys.phys_accs[ACC_X]);
        acc_samples[1].push_back(navdata_phys.phys_accs[ACC_Y]);
        acc_samples[2].push_back(navdata_phys.phys_accs[ACC_Z]);
        gyro_samples[0].push_back(navdata_phys.phys_gyros[GYRO_X]);
        gyro_samples[1].push_back(navdata_phys.phys_gyros[GYRO_Y]);
        gyro_samples[2].push_back(navdata_phys.phys_gyros[GYRO_Z]);
        vel_samples[0].push_back(navdata.vx);
        vel_samples[1].push_back(navdata.vy);
        vel_samples[2].push_back(navdata.vz);
        if (acc_samples[0].size() == max_num_samples)
        {
            for (int j = 0; j < 3; j++)
            {
                acc_bias[j] = calcAverage(acc_samples[j]);
                gyro_bias[j] = calcAverage(gyro_samples[j]);
                vel_bias[j] = calcAverage(vel_samples[j]);
            }
            ROS_INFO("Bias in linear acceleration (mg): [%4.4lf, %4.4lf, %4.4lf]", acc_bias[0], acc_bias[1], acc_bias[2]);
            ROS_INFO("Bias in angular velocity (deg/s): [%4.4lf, %4.4lf, %4.4lf]", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
            ROS_INFO("Bias in linear velocity (mm/s): [%4.4lf, %4.4lf, %4.4lf]", vel_bias[0], vel_bias[1], vel_bias[2]);
            ROS_INFO("Above values (except z-axis accel) will be substracted from actual IMU data in `navdata` and `imu` topic.");
            ROS_INFO("This feature can be disabled using `do_imu_caliberation` parameter. Recaliberate using `imu_recalib` service.");
            caliberated = true;
        }
    }
    if ((do_caliberation) && (caliberated))
    {
        for (int j = 0; j < 3; j++)
        {
            if (j != 2) navdata_phys.phys_accs[j] -= acc_bias[j];
            navdata_phys.phys_gyros[j] -= gyro_bias[j];
        }
        navdata.vx -= vel_bias[0];
        navdata.vy -= vel_bias[1];
        navdata.vz -= vel_bias[2];

    }
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
    imu_msg.header.frame_id = droneFrameBase;
    imu_msg.header.stamp = ros::Time::now();

    // IMU - Linear Acc
    imu_msg.linear_acceleration.x = msg.ax * 9.8;
    imu_msg.linear_acceleration.y = msg.ay * 9.8;
    imu_msg.linear_acceleration.z = msg.az * 9.8;

    // IMU - Rotation Matrix
    btQuaternion q;
    q.setEulerZYX(msg.rotZ * _DEG2RAD, msg.rotY * _DEG2RAD, msg.rotX * _DEG2RAD);
    tf::quaternionTFToMsg(q, imu_msg.orientation);

    // IMU - Gyro (Gyro is being sent in deg/sec)
    // TODO: Should Gyro be added to Navdata?
    imu_msg.angular_velocity.x = navdata_phys.phys_gyros[GYRO_X] * DEG_TO_RAD;
    imu_msg.angular_velocity.y = -navdata_phys.phys_gyros[GYRO_Y] * DEG_TO_RAD;
    imu_msg.angular_velocity.z = -navdata_phys.phys_gyros[GYRO_Z] * DEG_TO_RAD;

    imu_pub.publish(imu_msg);
}

void ARDroneDriver::publish_tf()
{
    tf_base_front.stamp_ = ros::Time::now();
    tf_base_bottom.stamp_ = ros::Time::now();
    tf_broad.sendTransform(tf_base_front);
    tf_broad.sendTransform(tf_base_bottom);
}

bool ARDroneDriver::imuReCalibCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    if (!do_caliberation)
    {
        ROS_WARN("Automatic IMU Caliberation is not active. Activate first using `do_imu_caliberation` parameter");
        return false;
    }
    else
    {
        ROS_WARN("Recaliberating IMU, please do not move the drone for a couple of seconds.");
        resetCaliberation();
        return true;
    }
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

