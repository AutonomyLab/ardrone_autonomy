/**
Software License Agreement (BSD)

\file      ardrone_driver.cpp
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
#include <signal.h>
#include <string>
#include <algorithm>
#include <vector>

#include <ardrone_autonomy/ardrone_driver.h>
#include <ardrone_autonomy/teleop_twist.h>
#include <ardrone_autonomy/video.h>


////////////////////////////////////////////////////////////////////////////////
// class ARDroneDriver
////////////////////////////////////////////////////////////////////////////////

ARDroneDriver::ARDroneDriver()
  : private_nh("~"),
    image_transport(node_handle),
    // Ugly: This has been defined in the template file. Cleaner way to guarantee initilaztion?
    initialized_navdata_publishers(false),
    last_frame_id(-1),
    last_navdata_id(-1),
    is_inited(false),
    last_receive_time(0.0)
{
  cmd_vel_sub = node_handle.subscribe("cmd_vel", 1, &CmdVelCallback);
  takeoff_sub = node_handle.subscribe("ardrone/takeoff", 1, &TakeoffCallback);
  reset_sub = node_handle.subscribe("ardrone/reset", 1, &ResetCallback);
  land_sub = node_handle.subscribe("ardrone/land", 1, &LandCallback);
  image_pub = image_transport.advertiseCamera("ardrone/image_raw", 10);
  hori_pub = image_transport.advertiseCamera("ardrone/front/image_raw", 10);
  vert_pub = image_transport.advertiseCamera("ardrone/bottom/image_raw", 10);
  toggle_cam_srv = node_handle.advertiseService("ardrone/togglecam", ToggleCamCallback);
  set_cam_channel_srv = node_handle.advertiseService("ardrone/setcamchannel", SetCamChannelCallback);
  set_led_animation_srv = node_handle.advertiseService("ardrone/setledanimation", SetLedAnimationCallback);
  flat_trim_srv = node_handle.advertiseService("ardrone/flattrim", FlatTrimCallback);
  set_flight_anim_srv = node_handle.advertiseService("ardrone/setflightanimation", SetFlightAnimationCallback);
  set_record_srv = node_handle.advertiseService("ardrone/setrecord", SetRecordCallback);

  /* TF Frames */
  std::string tf_prefix_key;
  private_nh.searchParam("tf_prefix", tf_prefix_key);
  private_nh.param(tf_prefix_key, tf_prefix, std::string(""));
  private_nh.param<std::string>("drone_frame_id", drone_frame_id, "ardrone_base");
  drone_frame_base = tf::resolve(tf_prefix, drone_frame_id + "_link");
  drone_frame_imu = tf::resolve(tf_prefix, drone_frame_id + "_imu");
  drone_frame_front_cam = tf::resolve(tf_prefix, drone_frame_id + "_frontcam");
  drone_frame_bottom_cam = tf::resolve(tf_prefix, drone_frame_id + "_bottomcam");
  tf_odom.frame_id_ = tf::resolve(tf_prefix, "odom");

  if (private_nh.hasParam("root_frame"))
  {
    ROS_WARN("Changing `root_frame` has been deprecated since version 1.4. ");
  }

  // Fill constant parts of IMU Message
  // If no rosparam is set then the default value of 0.0 will be assigned to all covariance values
  for (int i = 0; i < 9; i++)
  {
    imu_msg.linear_acceleration_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.orientation_covariance[i] = 0.0;
  }
  ReadCovParams("~cov/imu_la", imu_msg.linear_acceleration_covariance);
  ReadCovParams("~cov/imu_av", imu_msg.angular_velocity_covariance);
  ReadCovParams("~cov/imu_or", imu_msg.orientation_covariance);

  if (private_nh.hasParam("do_imu_caliberation"))
  {
    ROS_WARN("IMU Caliberation has been deprecated since 1.4.");
  }

  // Camera Info Manager
  cinfo_hori = new camera_info_manager::CameraInfoManager(ros::NodeHandle("ardrone/front"), "ardrone_front");
  cinfo_vert = new camera_info_manager::CameraInfoManager(ros::NodeHandle("ardrone/bottom"), "ardrone_bottom");

  // TF Stuff


  // Front Cam to Base
  // TODO(mani-monaj): Precise values for Drone1 & Drone2
  tf_base_front = tf::StampedTransform(
                    tf::Transform(
                      tf::createQuaternionFromRPY(-90.0 * _DEG2RAD, 0.0, -90.0 * _DEG2RAD),
                      tf::Vector3(0.21, 0.0, 0.0)),
                    ros::Time::now(), drone_frame_base, drone_frame_front_cam);

  // Bottom Cam to Base (Bad Assumption: No translation from IMU and Base)
  // TODO(mani-monaj): This should be different from Drone 1 & 2.
  tf_base_bottom = tf::StampedTransform(
                     tf::Transform(
                       tf::createQuaternionFromRPY(180.0 * _DEG2RAD, 0.0, 90.0 * _DEG2RAD),
                       tf::Vector3(0.0, -0.02, 0.0)),
                     ros::Time::now(), drone_frame_base, drone_frame_bottom_cam);

  // reset odometry
  odometry[0] = odometry[1] = 0;
}

ARDroneDriver::~ARDroneDriver()
{
  delete cinfo_hori;
  delete cinfo_vert;
}

void ARDroneDriver::run()
{
  ros::Rate loop_rate(looprate);
  const ros::Time startTime = ros::Time::now();
  static int freq_dev = 0;
  while (node_handle.ok())
  {
    if (!is_inited)  // Give the Drone 5s of free time to finish init phase
    {
      if (((ros::Time::now() - startTime).toSec()) > 5.0)
      {
        is_inited = true;

        // Send the configuration to the drone
        ConfigureDrone();

        vp_os_mutex_lock(&navdata_lock);
        ROS_INFO("Successfully connected to '%s' (AR-Drone %d.0 - Firmware: %s) - Battery(%%): %d",
                 ardrone_control_config.ardrone_name,
                 (IS_ARDRONE1) ? 1 : 2,
                 ardrone_control_config.num_version_soft,
                 shared_raw_navdata_ptr->navdata_demo.vbat_flying_percentage);
        ROS_INFO("Navdata Publish Settings:");
        ROS_INFO("    Legacy Navdata Mode: %s", enabled_legacy_navdata ? "On" : "Off");
        ROS_INFO("    ROS Loop Rate: %d Hz", looprate);
        ROS_INFO("    Realtime Navdata Publish: %s", realtime_navdata ? "On" : "Off");
        ROS_INFO("    Realtime Video Publish: %s", realtime_video ? "On" : "Off");
        ROS_INFO("    Drone Navdata Send Speed: %s",
                 ardrone_application_default_config.navdata_demo == 0 ?
                   "200Hz (navdata_demo=0)" : "15Hz (navdata_demo=1)");
        // TODO(mani-monaj): Enabled Navdata Demo
        vp_os_mutex_unlock(&navdata_lock);
        if (ardrone_control_config.num_version_soft[0] == '0')
        {
          ROS_WARN("The AR-Drone has a suspicious Firmware number. It usually means the network link is unreliable.");
        }
        ROS_DEBUG_STREAM("Using " << tf_prefix << " to prefix TF frames.");
      }
    }
    else
    {
      if (!realtime_video)
      {
        vp_os_mutex_lock(&video_lock);
        copy_current_frame_id = current_frame_id;
        vp_os_mutex_unlock(&video_lock);
        if (copy_current_frame_id != last_frame_id)
        {
          last_frame_id = copy_current_frame_id;
          PublishVideo();
        }
      }

      if (!realtime_navdata)
      {
        vp_os_mutex_lock(&navdata_lock);
        copy_current_navdata_id = current_navdata_id;
        vp_os_mutex_unlock(&navdata_lock);
        if (copy_current_navdata_id != last_navdata_id)
        {
          vp_os_mutex_lock(&navdata_lock);
          last_navdata_id = copy_current_navdata_id;

          // Thread safe copy of interesting Navdata data
          // TODO(mani-monaj): This is a very expensive task, can we optimize here?
          // maybe ignoring the copy when it is not needed.
          const navdata_unpacked_t navdata_raw = *shared_raw_navdata_ptr;
          const ros::Time navdata_receive_time = shared_navdata_receive_time;
          vp_os_mutex_unlock(&navdata_lock);

          // This function is defined in the template NavdataMessageDefinitions.h template file
          PublishNavdataTypes(navdata_raw, navdata_receive_time);
          PublishNavdata(navdata_raw, navdata_receive_time);
          PublishOdometry(navdata_raw, navdata_receive_time);
        }
      }
      if (freq_dev == 0) PublishTF();

      // (looprate / 5)Hz  TF publish
      // TODO(mani-monaj): Make TF publish rate fixed
      freq_dev = (freq_dev + 1) % 5;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  printf("ROS loop terminated ... \n");
}

void ARDroneDriver::ConfigureDrone()
{
  #include "ardrone_autonomy/snippet_configure_drone.h"
}

double ARDroneDriver::CalcAverage(const std::vector<double> &vec)
{
  double ret = 0.0;
  for (unsigned int i = 0; i < vec.size(); i++)
  {
    ret += vec[i];
  }
  return (ret / vec.size());
}

bool ARDroneDriver::ReadCovParams(const std::string &param_name, boost::array<double, 9> &cov_array)
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
      cov_array[i] = static_cast<double>(cov_list[i]);
      str_stream << cov_array[i] << ((i < 8) ? ", " : "");
    }
    str_stream << "]";
    ROS_INFO("%s", str_stream.str().c_str());
    return true;
  }
  else
  {
    ROS_INFO("No values found for `%s` in parameters, set all to zero.", param_name.c_str());
    return false;
  }
}

void ARDroneDriver::PublishVideo()
{
  if (
    (image_pub.getNumSubscribers() == 0) &&
    (hori_pub.getNumSubscribers() == 0) &&
    (vert_pub.getNumSubscribers() == 0)
  ) return;

  // Camera Info (NO PIP)

  sensor_msgs::CameraInfo cinfo_msg_hori = cinfo_hori->getCameraInfo();
  sensor_msgs::CameraInfo cinfo_msg_vert = cinfo_vert->getCameraInfo();

  cinfo_msg_hori.header.frame_id = drone_frame_front_cam;
  cinfo_msg_vert.header.frame_id = drone_frame_bottom_cam;

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

    if ((cam_state == ZAP_CHANNEL_HORI) || (cam_state == ZAP_CHANNEL_LARGE_HORI_SMALL_VERT))
    {
      image_msg.header.frame_id = drone_frame_front_cam;
    }
    else if ((cam_state == ZAP_CHANNEL_VERT) || (cam_state == ZAP_CHANNEL_LARGE_VERT_SMALL_HORI))
    {
      image_msg.header.frame_id = drone_frame_bottom_cam;
    }
    else
    {
      ROS_WARN_ONCE("Something is wrong with camera channel config.");
    }

    image_msg.width = D1_STREAM_WIDTH;
    image_msg.height = D1_STREAM_HEIGHT;
    image_msg.encoding = "rgb8";
    image_msg.is_bigendian = false;
    image_msg.step = D1_STREAM_WIDTH * 3;
    image_msg.data.resize(D1_STREAM_WIDTH * D1_STREAM_HEIGHT * 3);

    if (!realtime_video) vp_os_mutex_lock(&video_lock);
    image_msg.header.stamp = shared_video_receive_time;
    std::copy(buffer, buffer + (D1_STREAM_WIDTH * D1_STREAM_HEIGHT * 3), image_msg.data.begin());
    if (!realtime_video) vp_os_mutex_unlock(&video_lock);

    cinfo_msg_hori.header.stamp = image_msg.header.stamp;
    cinfo_msg_vert.header.stamp = image_msg.header.stamp;

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
      image_msg.step = D1_VERTSTREAM_WIDTH * 3;
      image_msg.data.clear();
      image_msg.data.resize(D1_VERTSTREAM_WIDTH * D1_VERTSTREAM_HEIGHT * 3);
      _it = image_msg.data.begin();
      if (!realtime_video) vp_os_mutex_lock(&video_lock);
      for (int row = 0; row < D1_VERTSTREAM_HEIGHT ; row++)
      {
        int _b = row * D1_STREAM_WIDTH * 3;
        int _e = _b + image_msg.step;
        _it = std::copy(buffer + _b, buffer + _e, _it);
      }
      if (!realtime_video) vp_os_mutex_unlock(&video_lock);

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
      image_msg.step = (D1_STREAM_WIDTH - D1_MODE2_PIP_WIDTH) * 3;
      image_msg.data.clear();
      image_msg.data.resize((D1_STREAM_WIDTH - D1_MODE2_PIP_WIDTH)*D1_STREAM_HEIGHT * 3);
      _it = image_msg.data.begin();
      if (!realtime_video) vp_os_mutex_lock(&video_lock);
      for (int row = 0; row < D1_STREAM_HEIGHT; row++)
      {
        int _b = (row * D1_STREAM_WIDTH * 3) + (D1_MODE2_PIP_WIDTH * 3);
        int _e = _b + image_msg.step;
        _it = std::copy(buffer + _b, buffer + _e, _it);
      }
      if (!realtime_video) vp_os_mutex_unlock(&video_lock);

      cinfo_msg_hori.width = D1_STREAM_WIDTH - D1_MODE2_PIP_WIDTH;
      cinfo_msg_hori.height = D1_STREAM_HEIGHT;
      hori_pub.publish(image_msg, cinfo_msg_hori);

      // Bottom
      image_msg.width = D1_MODE2_PIP_WIDTH;
      image_msg.height = D1_MODE2_PIP_HEIGHT;
      image_msg.encoding = "rgb8";
      image_msg.is_bigendian = false;
      image_msg.step = D1_MODE2_PIP_WIDTH * 3;
      image_msg.data.clear();
      image_msg.data.resize(D1_MODE2_PIP_WIDTH * D1_MODE2_PIP_HEIGHT * 3);
      _it = image_msg.data.begin();
      if (!realtime_video) vp_os_mutex_lock(&video_lock);
      for (int row = 0; row < D1_MODE2_PIP_HEIGHT; row++)
      {
        int _b = row * D1_STREAM_WIDTH * 3;
        int _e = _b + image_msg.step;
        _it = std::copy(buffer + _b, buffer + _e, _it);
      }
      if (!realtime_video) vp_os_mutex_unlock(&video_lock);

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
      image_msg.step = (D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH) * 3;
      image_msg.data.clear();
      image_msg.data.resize((D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH)* D1_VERTSTREAM_HEIGHT * 3);
      _it = image_msg.data.begin();
      for (int row = 0; row < D1_VERTSTREAM_HEIGHT; row++)
      {
        int _b = (row * (D1_STREAM_WIDTH * 3)) + (D1_MODE3_PIP_WIDTH * 3);
        int _e = _b + image_msg.step;
        if (!realtime_video) vp_os_mutex_lock(&video_lock);
        _it = std::copy(buffer + _b, buffer + _e, _it);
        if (!realtime_video) vp_os_mutex_unlock(&video_lock);
      }

      cinfo_msg_vert.width = D1_VERTSTREAM_WIDTH - D1_MODE3_PIP_WIDTH;
      cinfo_msg_vert.height = D1_VERTSTREAM_HEIGHT;
      vert_pub.publish(image_msg, cinfo_msg_vert);

      // Front
      image_msg.width = D1_MODE3_PIP_WIDTH;
      image_msg.height = D1_MODE3_PIP_HEIGHT;
      image_msg.encoding = "rgb8";
      image_msg.is_bigendian = false;
      image_msg.step = D1_MODE3_PIP_WIDTH * 3;
      image_msg.data.clear();
      image_msg.data.resize(D1_MODE3_PIP_WIDTH * D1_MODE3_PIP_HEIGHT * 3);
      _it = image_msg.data.begin();
      if (!realtime_video) vp_os_mutex_lock(&video_lock);
      for (int row = 0; row < D1_MODE3_PIP_HEIGHT; row++)
      {
        int _b = row * D1_STREAM_WIDTH * 3;
        int _e = _b + image_msg.step;
        _it = std::copy(buffer + _b, buffer + _e, _it);
      }
      if (!realtime_video) vp_os_mutex_unlock(&video_lock);

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

    if (cam_state == ZAP_CHANNEL_HORI)
    {
      image_msg.header.frame_id = drone_frame_front_cam;
    }
    else if (cam_state == ZAP_CHANNEL_VERT)
    {
      image_msg.header.frame_id = drone_frame_bottom_cam;
    }
    else
    {
      ROS_WARN_ONCE("Something is wrong with camera channel config.");
    }

    image_msg.width = D2_STREAM_WIDTH;
    image_msg.height = D2_STREAM_HEIGHT;
    image_msg.encoding = "rgb8";
    image_msg.is_bigendian = false;
    image_msg.step = D2_STREAM_WIDTH * 3;
    image_msg.data.resize(D2_STREAM_WIDTH * D2_STREAM_HEIGHT * 3);
    if (!realtime_video) vp_os_mutex_lock(&video_lock);
    image_msg.header.stamp = shared_video_receive_time;
    std::copy(buffer, buffer + (D2_STREAM_WIDTH * D2_STREAM_HEIGHT * 3), image_msg.data.begin());
    if (!realtime_video) vp_os_mutex_unlock(&video_lock);
    // We only put the width and height in here.

    cinfo_msg_hori.header.stamp = image_msg.header.stamp;
    cinfo_msg_vert.header.stamp = image_msg.header.stamp;

    if (cam_state == ZAP_CHANNEL_HORI)
    {
      /*
      * Horizontal camera is activated, only /ardrone/front/ is being updated
      */
      cinfo_msg_hori.width = D2_STREAM_WIDTH;
      cinfo_msg_hori.height = D2_STREAM_HEIGHT;
      image_pub.publish(image_msg, cinfo_msg_hori);  // /ardrone
      hori_pub.publish(image_msg, cinfo_msg_hori);
    }
    else if (cam_state == ZAP_CHANNEL_VERT)
    {
      /*
      * Vertical camera is activated, only /ardrone/bottom/ is being updated
      */
      cinfo_msg_vert.width = D2_STREAM_WIDTH;
      cinfo_msg_vert.height = D2_STREAM_HEIGHT;
      image_pub.publish(image_msg, cinfo_msg_vert);  // /ardrone
      vert_pub.publish(image_msg, cinfo_msg_vert);
    }
  }
}

void ARDroneDriver::PublishNavdata(const navdata_unpacked_t &navdata_raw, const ros::Time &navdata_receive_time)
{
  if (!enabled_legacy_navdata ||
      ((navdata_pub.getNumSubscribers() == 0) &&
       (imu_pub.getNumSubscribers() == 0) &&
       (mag_pub.getNumSubscribers() == 0)))
    return;  // why bother, no one is listening.

  legacynavdata_msg.header.stamp = navdata_receive_time;
  legacynavdata_msg.header.frame_id = drone_frame_base;
  legacynavdata_msg.batteryPercent = navdata_raw.navdata_demo.vbat_flying_percentage;
  legacynavdata_msg.state = (navdata_raw.navdata_demo.ctrl_state >> 16);

  // positive means counterclockwise rotation around axis
  legacynavdata_msg.rotX = navdata_raw.navdata_demo.phi / 1000.0;  // tilt left/right
  legacynavdata_msg.rotY = -navdata_raw.navdata_demo.theta / 1000.0;  // tilt forward/backward
  legacynavdata_msg.rotZ = -navdata_raw.navdata_demo.psi / 1000.0;  // orientation

  legacynavdata_msg.altd = navdata_raw.navdata_demo.altitude;  // cm
  legacynavdata_msg.vx = navdata_raw.navdata_demo.vx;  // mm/sec
  legacynavdata_msg.vy = -navdata_raw.navdata_demo.vy;  // mm/sec
  legacynavdata_msg.vz = -navdata_raw.navdata_demo.vz;  // mm/sec
  // msg.tm = navdata_raw.navdata_time.time;
  // First 21 bits (LSB) are usecs + 11 HSB are seconds
  legacynavdata_msg.tm = (navdata_raw.navdata_time.time & 0x001FFFFF) + (navdata_raw.navdata_time.time >> 21) * 1000000;
  legacynavdata_msg.ax = navdata_raw.navdata_phys_measures.phys_accs[ACC_X] / 1000.0;  // g
  legacynavdata_msg.ay = -navdata_raw.navdata_phys_measures.phys_accs[ACC_Y] / 1000.0;  // g
  legacynavdata_msg.az = -navdata_raw.navdata_phys_measures.phys_accs[ACC_Z] / 1000.0;  // g

  legacynavdata_msg.motor1 = navdata_raw.navdata_pwm.motor1;
  legacynavdata_msg.motor2 = navdata_raw.navdata_pwm.motor2;
  legacynavdata_msg.motor3 = navdata_raw.navdata_pwm.motor3;
  legacynavdata_msg.motor4 = navdata_raw.navdata_pwm.motor4;

  // New stuff

  if (IS_ARDRONE2)
  {
    legacynavdata_msg.magX = (int32_t)navdata_raw.navdata_magneto.mx;
    legacynavdata_msg.magY = (int32_t)navdata_raw.navdata_magneto.my;
    legacynavdata_msg.magZ = (int32_t)navdata_raw.navdata_magneto.mz;

    legacynavdata_msg.pressure = navdata_raw.navdata_pressure_raw.Pression_meas;  // typo in the SDK!
    legacynavdata_msg.temp = navdata_raw.navdata_pressure_raw.Temperature_meas;

    legacynavdata_msg.wind_speed = navdata_raw.navdata_wind_speed.wind_speed;
    legacynavdata_msg.wind_angle = navdata_raw.navdata_wind_speed.wind_angle;
    legacynavdata_msg.wind_comp_angle = navdata_raw.navdata_wind_speed.wind_compensation_phi;
  }
  else
  {
    legacynavdata_msg.magX = legacynavdata_msg.magY = legacynavdata_msg.magZ = 0;
    legacynavdata_msg.pressure = 0.0;
    legacynavdata_msg.temp = 0.0;
    legacynavdata_msg.wind_speed = 0.0;
    legacynavdata_msg.wind_angle = 0.0;
    legacynavdata_msg.wind_comp_angle = 0.0;
  }

  // Tag Detection, need to clear vectors first because it's a member variable now
  legacynavdata_msg.tags_type.clear();
  legacynavdata_msg.tags_xc.clear();
  legacynavdata_msg.tags_yc.clear();
  legacynavdata_msg.tags_width.clear();
  legacynavdata_msg.tags_height.clear();
  legacynavdata_msg.tags_orientation.clear();
  legacynavdata_msg.tags_distance.clear();

  legacynavdata_msg.tags_count = navdata_raw.navdata_vision_detect.nb_detected;
  for (int i = 0; i < navdata_raw.navdata_vision_detect.nb_detected; i++)
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
    legacynavdata_msg.tags_type.push_back(navdata_raw.navdata_vision_detect.type[i]);
    legacynavdata_msg.tags_xc.push_back(navdata_raw.navdata_vision_detect.xc[i]);
    legacynavdata_msg.tags_yc.push_back(navdata_raw.navdata_vision_detect.yc[i]);
    legacynavdata_msg.tags_width.push_back(navdata_raw.navdata_vision_detect.width[i]);
    legacynavdata_msg.tags_height.push_back(navdata_raw.navdata_vision_detect.height[i]);
    legacynavdata_msg.tags_orientation.push_back(navdata_raw.navdata_vision_detect.orientation_angle[i]);
    legacynavdata_msg.tags_distance.push_back(navdata_raw.navdata_vision_detect.dist[i]);
  }

  /* IMU */
  imu_msg.header.frame_id = drone_frame_base;
  imu_msg.header.stamp = navdata_receive_time;

  // IMU - Linear Acc
  imu_msg.linear_acceleration.x = legacynavdata_msg.ax * 9.8;
  imu_msg.linear_acceleration.y = legacynavdata_msg.ay * 9.8;
  imu_msg.linear_acceleration.z = legacynavdata_msg.az * 9.8;

  // IMU - Rotation Matrix
  tf::Quaternion q;
  q.setRPY(legacynavdata_msg.rotX * _DEG2RAD, legacynavdata_msg.rotY * _DEG2RAD, legacynavdata_msg.rotZ * _DEG2RAD);
  tf::quaternionTFToMsg(q, imu_msg.orientation);

  // IMU - Gyro (Gyro is being sent in deg/sec)
  // TODO(mani-monaj): Should Gyro be added to Navdata?
  imu_msg.angular_velocity.x = navdata_raw.navdata_phys_measures.phys_gyros[GYRO_X] * DEG_TO_RAD;
  imu_msg.angular_velocity.y = -navdata_raw.navdata_phys_measures.phys_gyros[GYRO_Y] * DEG_TO_RAD;
  imu_msg.angular_velocity.z = -navdata_raw.navdata_phys_measures.phys_gyros[GYRO_Z] * DEG_TO_RAD;

  mag_msg.header.frame_id = drone_frame_base;
  mag_msg.header.stamp = navdata_receive_time;
  const float mag_normalizer = sqrt(legacynavdata_msg.magX * legacynavdata_msg.magX +
                                    legacynavdata_msg.magY * legacynavdata_msg.magY +
                                    legacynavdata_msg.magZ * legacynavdata_msg.magZ);

  // TODO(mani-monaj): Check if it is really needed that magnetometer message includes normalized value
  if (fabs(mag_normalizer) > 1e-9f)
  {
    mag_msg.vector.x = legacynavdata_msg.magX / mag_normalizer;
    mag_msg.vector.y = legacynavdata_msg.magY / mag_normalizer;
    mag_msg.vector.z = legacynavdata_msg.magZ / mag_normalizer;
    mag_pub.publish(mag_msg);
  }
  else
  {
    ROS_WARN_THROTTLE(30, "There is something wrong with the magnetometer readings (Magnitude is extremely small).");
  }

  navdata_pub.publish(legacynavdata_msg);
  imu_pub.publish(imu_msg);
}

// Load actual auto-generated code to publish full navdata
#define NAVDATA_STRUCTS_SOURCE
#include <ardrone_autonomy/NavdataMessageDefinitions.h>
#undef NAVDATA_STRUCTS_SOURCE

void ARDroneDriver::PublishTF()
{
  tf_base_front.stamp_ = ros::Time::now();
  tf_base_bottom.stamp_ = ros::Time::now();
  tf_broad.sendTransform(tf_base_front);
  tf_broad.sendTransform(tf_base_bottom);
}

void ARDroneDriver::PublishOdometry(const navdata_unpacked_t &navdata_raw, const ros::Time &navdata_receive_time)
{
  if (last_receive_time.isValid())
  {
    double delta_t = (navdata_receive_time - last_receive_time).toSec();
    odometry[0] += ((cos((navdata_raw.navdata_demo.psi / 180000.0) * M_PI) *
                     navdata_raw.navdata_demo.vx - sin((navdata_raw.navdata_demo.psi / 180000.0) * M_PI) *
                     -navdata_raw.navdata_demo.vy) * delta_t) / 1000.0;
    odometry[1] += ((sin((navdata_raw.navdata_demo.psi / 180000.0) * M_PI) *
                     navdata_raw.navdata_demo.vx + cos((navdata_raw.navdata_demo.psi / 180000.0) * M_PI) *
                     -navdata_raw.navdata_demo.vy) * delta_t) / 1000.0;
  }
  last_receive_time = navdata_receive_time;

  nav_msgs::Odometry odo_msg;
  odo_msg.header.stamp = navdata_receive_time;
  odo_msg.header.frame_id = "odom";
  odo_msg.child_frame_id = drone_frame_base;

  odo_msg.pose.pose.position.x = odometry[0];
  odo_msg.pose.pose.position.y = odometry[1];
  odo_msg.pose.pose.position.z = navdata_raw.navdata_demo.altitude / 1000.0;
  odo_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
        navdata_raw.navdata_demo.phi / 180000.0 * M_PI,
        -navdata_raw.navdata_demo.theta / 180000.0 * M_PI,
        -navdata_raw.navdata_demo.psi / 180000.0 * M_PI);

  odo_msg.twist.twist.linear.x = navdata_raw.navdata_demo.vx / 1000.0;
  odo_msg.twist.twist.linear.y = -navdata_raw.navdata_demo.vy / 1000.0;
  odo_msg.twist.twist.linear.z = -navdata_raw.navdata_demo.vz / 1000.0;

  if (odo_pub.getNumSubscribers() > 0)
  {
    odo_pub.publish(odo_msg);
  }

  tf::Vector3 t;
  tf::pointMsgToTF(odo_msg.pose.pose.position, t);
  tf::Quaternion q;
  tf::quaternionMsgToTF(odo_msg.pose.pose.orientation, q);

  tf_odom.stamp_ = navdata_receive_time;
  tf_odom.child_frame_id_ = drone_frame_base;
  tf_odom.setOrigin(t);
  tf_odom.setRotation(q);
  tf_broad.sendTransform(tf_odom);
}

void ControlCHandler(int signal)
{
  ros::shutdown();
  should_exit = 1;
}

////////////////////////////////////////////////////////////////////////////////
// custom_main
////////////////////////////////////////////////////////////////////////////////

// extern "C" int custom_main(int argc, char** argv)
int main(int argc, char** argv)
{
  C_RESULT res = C_FAIL;
  char * drone_ip_address = NULL;

  // We need to implement our own Signal handler instead of ROS to shutdown
  // the SDK threads correctly.

  ros::init(argc, argv, "ardrone_driver", ros::init_options::NoSigintHandler);

  signal(SIGABRT, &ControlCHandler);
  signal(SIGTERM, &ControlCHandler);
  signal(SIGINT, &ControlCHandler);

  // Now to setup the drone and communication channels
  // We do this here because calling ardrone_tool_main uses an old
  // function initialization and is no longer recommended by parrot
  // I've based this section off the ControlEngine's initialization
  // routine (distributed with ARDrone SDK 2.0 Examples) as well as
  // the ardrone_tool_main function

  // Parse command line for
  // Backward compatibility with `-ip` command line argument
  argc--;
  argv++;
  while (argc && *argv[0] == '-')
  {
    if (!strcmp(*argv, "-ip") && (argc > 1))
    {
      drone_ip_address = *(argv + 1);
      printf("Using custom ip address %s\n", drone_ip_address);
      argc--;
      argv++;
    }
    argc--;
    argv++;
  }

  // Configure wifi
  vp_com_wifi_config_t *config = reinterpret_cast<vp_com_wifi_config_t*>(wifi_config());

  if (config)
  {
    vp_os_memset(&wifi_ardrone_ip[0], 0, ARDRONE_IPADDRESS_SIZE);

    // TODO(mani-monaj): Check if IP is valid
    if (drone_ip_address)
    {
      printf("===================+> %s\n", drone_ip_address);
      strncpy(&wifi_ardrone_ip[0], drone_ip_address, ARDRONE_IPADDRESS_SIZE - 1);
    }
    else
    {
      printf("===================+> %s\n", config->server);
      strncpy(&wifi_ardrone_ip[0], config->server, ARDRONE_IPADDRESS_SIZE - 1);
    }
  }

  while (-1 == getDroneVersion(".", wifi_ardrone_ip, &ardroneVersion))
  {
    printf("Getting AR.Drone version ...\n");
    vp_os_delay(250);
  }

  // Setup communication channels
  res = ardrone_tool_setup_com(NULL);
  if (FAILED(res))
  {
    PRINT("Wifi initialization failed. It means either:\n");
    PRINT("\t* you're not root (it's mandatory because you can set up wifi connection only as root)\n");
    PRINT("\t* wifi device is not present (on your pc or on your card)\n");
    PRINT("\t* you set the wrong name for wifi interface (for example rausb0 instead of wlan0) \n");
    PRINT("\t* ap is not up (reboot card or remove wifi usb dongle)\n");
    PRINT("\t* wifi device has no antenna\n");
  }
  else
  {
    // setup the application and user profiles for the driver
    const char* appname = reinterpret_cast<const char*>(DRIVER_APPNAME);
    const char* usrname = reinterpret_cast<const char*>(DRIVER_USERNAME);
    ardrone_gen_appid(appname, "2.0", app_id, app_name, APPLI_NAME_SIZE);
    ardrone_gen_usrid(usrname, usr_id, usr_name, USER_NAME_SIZE);

    // and finally initialize everything!
    // this will then call our sdk, which then starts the ::run() method of this file as an ardrone client thread

    res = ardrone_tool_init(wifi_ardrone_ip,
                            strlen(wifi_ardrone_ip),
                            NULL,
                            app_name,
                            usr_name,
                            NULL,
                            NULL,
                            MAX_FLIGHT_STORING_SIZE,
                            NULL);

    while (SUCCEED(res) && ardrone_tool_exit() == FALSE)
    {
      res = ardrone_tool_update();
    }
    res = ardrone_tool_shutdown();
  }
  return SUCCEED(res) ? 0 : -1;
}
