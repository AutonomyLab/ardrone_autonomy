/**
Software License Agreement (BSD)

\file      ardrone_driver.h
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
#ifndef ARDRONE_AUTONOMY_ARDRONE_DRIVER_H
#define ARDRONE_AUTONOMY_ARDRONE_DRIVER_H

class ARDroneDriver;

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/ardrone_sdk.h>

// C/C++
#include <stdint.h>
#include <vector>
#include <string>

// ardronelib
#include <utils/ardrone_gen_ids.h>
#include <ardrone_tool/ardrone_version.h>
#include <ardrone_tool/ardrone_tool.h>

// Load auto-generated include files for full navdata
#define NAVDATA_STRUCTS_INCLUDES
#include <ardrone_autonomy/NavdataMessageDefinitions.h>
#undef NAVDATA_STRUCTS_INCLUDES

#define _DEG2RAD 0.01745331111
#define _RAD2DEG 57.2957184819

#define DRIVER_USERNAME "ardrone_driver"
#define DRIVER_APPNAME "ardrone_driver"
#define CAMERA_QUEUE_SIZE (30)
#define NAVDATA_QUEUE_SIZE (200)

class ARDroneDriver
{
public:
  ARDroneDriver();
  ~ARDroneDriver();
  void run();
  static bool ReadCovParams(const std::string& param_name, boost::array<double, 9> &cov_array);
  static double CalcAverage(const std::vector<double> &vec);
  void PublishVideo();
  void PublishNavdata(const navdata_unpacked_t &navdata_raw, const ros::Time &navdata_receive_time);
  void PublishOdometry(const navdata_unpacked_t &navdata_raw, const ros::Time &navdata_receive_time);

#define NAVDATA_STRUCTS_HEADER_PUBLIC
#include <ardrone_autonomy/NavdataMessageDefinitions.h>
#undef NAVDATA_STRUCTS_HEADER_PUBLIC

private:
  void PublishTF();
  void ConfigureDrone();

  ros::NodeHandle node_handle;
  ros::NodeHandle private_nh;
  ros::Subscriber cmd_vel_sub;
  ros::Subscriber takeoff_sub;
  ros::Subscriber reset_sub;
  ros::Subscriber land_sub;
  image_transport::ImageTransport image_transport;
  image_transport::CameraPublisher image_pub;
  image_transport::CameraPublisher hori_pub;
  image_transport::CameraPublisher vert_pub;

  camera_info_manager::CameraInfoManager *cinfo_hori;
  camera_info_manager::CameraInfoManager *cinfo_vert;

  ros::Publisher navdata_pub;
  ros::Publisher imu_pub;
  ros::Publisher mag_pub;
  ros::Publisher odo_pub;

  tf::TransformBroadcaster tf_broad;

  ros::ServiceServer toggle_cam_srv;
  ros::ServiceServer set_cam_channel_srv;
  ros::ServiceServer set_led_animation_srv;
  ros::ServiceServer flat_trim_srv;
  ros::ServiceServer set_flight_anim_srv;
  ros::ServiceServer set_record_srv;

  int32_t last_frame_id;
  int32_t last_navdata_id;
  int32_t copy_current_frame_id;
  int32_t copy_current_navdata_id;

  int16_t flying_state;

  bool is_inited;
  std::string drone_frame_id;

  // Load auto-generated declarations for full navdata
#define NAVDATA_STRUCTS_HEADER_PRIVATE
#include <ardrone_autonomy/NavdataMessageDefinitions.h>
#undef NAVDATA_STRUCTS_HEADER_PRIVATE

  /*
   * TF Frames
   * Base: Should be COM
   */
  std::string tf_prefix, drone_frame_base, drone_frame_imu, drone_frame_front_cam, drone_frame_bottom_cam;
  tf::StampedTransform tf_base_front, tf_base_bottom, tf_odom;

  // Huge part of IMU message is constant, let's fill'em once.
  sensor_msgs::Imu imu_msg;
  geometry_msgs::Vector3Stamped mag_msg;
  ardrone_autonomy::Navdata legacynavdata_msg;

  // odometry (x,y)
  ros::Time last_receive_time;
  double odometry[2];
};

#endif  // ARDRONE_AUTONOMY_ARDRONE_DRIVER_H
