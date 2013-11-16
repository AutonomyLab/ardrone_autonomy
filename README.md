# ardrone_autonomy : A ROS Driver for ARDrone 1.0 & 2.0

"ardrone_autonomy" is a [ROS](http://ros.org/ "Robot Operating System") driver for [Parrot AR-Drone](http://http://ardrone.parrot.com/parrot-ar-drone/select-site) quadrocopter. This driver is based on official [AR-Drone SDK](https://projects.ardrone.org/) version 2.0 and supports both AR-Drone 1.0 and 2.0. "ardrone_autonomy" is a fork of [AR-Drone Brown](http://code.google.com/p/brown-ros-pkg/wiki/ardrone_brown) driver. This package has been developed in [Autonomy Lab](http://autonomy.cs.sfu.ca) of [Simon Fraser University](http://www.sfu.ca) by [Mani Monajjemi](http://sfu.ca/~mmonajje) ( +other [contributors](#contributors)).


## Table of Contents

- [Updates](#updates)
- [Installation](#installation)
	- [Pre-requirements](#pre-requirements)
	- [Installation Steps](#installation-steps)
- [How to Run](#how-to-run)
- [Reading from AR-Drone](#reading-from-ar-drone)
	- [Update Frequencies ](#update-frequencies-new)
	- [Legacy Navigation Data](#legacy-navigation-data)
	- [IMU data](#imu-data)
	- [Magnetometer Data](#magnetometer-data)
	- [Selective Navdata (Advanced) ](#selective-navdata-advanced-new)
	- [Cameras](#cameras)
	- [Tag Detection](#tag-detection)
	- [Update Frequencies](#update-frequencies)
- [Sending Commands to AR-Drone](#sending-commands-to-ar-drone)
	- [Hover Modes ](#hover-modes-new) :new:
- [Coordinate Frames](#coordinate-frames)
- [Services](#services)
	- [Toggle AR-Drone's Camera](#toggle-ar-drones-camera)
	- [LED Animations](#led-animations)
	- [Flight Animations ](#flight-animations-new)
	- [IMU Calibration](#imu-calibration)
	- [Flat Trim](#flat-trim)
    - [Record to USB Stick](#record-to-usb-stick) :new:
- [Parameters](#parameters)
	- [AR-Drone Specific Parameters](#ar-drone-specific-parameters)
	- [Other Parameters](#other-parameters)
- [License](#license)
- [Contributors](#contributors)
- [FAQ](#faq) :new:



## Updates

- *October 22 2013*: Update to Parrot SDK 2.0.1 (Fixes crashes on 2.4.x firmwares, no support for flight recorder (yet). **Please check the FAQ section for instructions on how to re-compile the SDK**. (Tested on 2.3.3 and 2.4.x firmwares) 
- *February 13 2013*: Support for USB key recording ([More info](https://github.com/AutonomyLab/ardrone_autonomy/pull/53)). Motor PWM added to legacy Navdata.
- *January 9 2013*: ROS Groovy support. Support for zero-command without hovering ([More info](https://github.com/AutonomyLab/ardrone_autonomy/pull/34)). Full configurable Navdata support ([More info](https://github.com/AutonomyLab/ardrone_autonomy/pull/31)). Support for "Flight Animations". Support for Real-time navdata and video publishing ([More info](https://github.com/AutonomyLab/ardrone_autonomy/pull/44)). Support for configurable data publishing rate.
- *November 9 2012*: Critical Bug in sending configurations to drone fixed and more parameters are supported ([More info](https://github.com/AutonomyLab/ardrone_autonomy/issues/24)). Separate topic for magnetometer data added ([More info](https://github.com/AutonomyLab/ardrone_autonomy/pull/25)).
- *September 5 2012*: Experimental automatic IMU bias removal.
- *August 27 2012*: Thread-safe SDK data access. Synchronized `navdata` and `camera` topics.
- *August 20 2012*: The driver is now provides ROS standard camera interface.
- *August 17 2012*: Experimental `tf` support added. New published topic `imu`.
- *August 1 2012*: Enhanced `Navdata` message. `Navdata` now includes magnetometer data, barometer data, temperature and wind information for AR-Drone 2. [Issue #2](https://github.com/AutonomyLab/ardrone_autonomy/pull/2)
- *July 27 2012*: LED Animation Support added to the driver as a service
- *July 19 2012*: Initial Public Release

## Installation

### Pre-requirements

This driver has been tested on Linux machines running Ubuntu 11.10, 12.04 & 12.10 (32 bit and 64 bit). However it should also work on any other mainstream Linux distribution. The driver has been tested on both ROS "electric" and "fuerte". The AR-Drone SDK has its own build system which usually handles system wide dependencies itself. The ROS package depends on these standard ROS packages: `roscpp`, `image_transport`, `sensor_msgs`, `tf`, `camera_info_manager` and `std_srvs`.

### Installation Steps

The installation follows the same steps needed usually to compile a ROS driver.

* Get the code: Clone (or download and unpack) the driver to your personal ROS stacks folder (e.g. ~/ros/stacks) and `cd` to it. Please make sure that this folder is in your `ROS_PACKAGE_PATH` environmental variable.

        ```bash
        $ cd ~/ros/stacks
        $ git clone https://github.com/AutonomyLab/ardrone_autonomy.git
        $ rosstack profile && rospack profile
        $ roscd ardrone_autonomy
        ```

**NOTE (For advanced users):** Instead of the `master` branch you can use the `dev-unstable` branch for the latest _unstable_ code which may contain bug fixes or new features. This is the branch that all developments happen on. Please use this branch to submit pull requests.
 
* Compile the AR-Drone SDK: The driver contains a slightly patched version of AR-Drone 2.0 SDK which is located in `ARDroneLib` directory. To compile it, execute the `./build_sdk.sh`. Any system-wide dependency will be managed by the SDK's build script. You may be asked to install some packages during the installation procedure (e.g `daemontools`). You can verify the success of the SDK's build by checking the `lib` folder.

        ```bash
        $ ./build_sdk.sh
        $ [After a couple of minutes]
        $ ls ./lib

        libavcodec.a   libavformat.a    libpc_ardrone_notool.a  libvlib.a
        libavdevice.a  libavutil.a      libsdk.a
        libavfilter.a  libpc_ardrone.a  libswscale.a
        ```

* Compile the driver: You can easily compile the driver by using `rosmake ardrone_autonomy` command.

## How to Run

The driver's executable node is `ardrone_driver`. You can either use `rosrun ardrone_autonomy ardrone_driver` or put it in a custom launch file with your desired parameters.

## Reading from AR-Drone

### Update Frequencies :new:

**Drone Update Frequencies**: The drone's data transmission update frequency depends on `navdata_demo` parameter. When it is 1, the transmission frequency will be 15Hz, otherwise it will be 200Hz. (`navdata_demo` is a numeric parameter not Boolean, so use 1 and 0 (not True/False) to set/unset it)

**Driver Update Frequencies**: The driver can operate in two modes: real-time or fixed rate. When the `realtime_navdata` parameter is set to True, the driver will publish the received information instantly. However when it is set to False, the driver will cache the most recent received data, then it will publish that at a fixed rate, configured by `looprate` parameter. The default configuration is: `realtime_navdata=False` and `looprate=50`. 

Please note that if the `looprate` is smaller than the drone's transmission frequency, there will be data loss. The driver's start-up output shows the current configuration. You can also use `rostopic hz` command to check the publish rate of the driver.

<pre>
# Default Setting - 50Hz non-realtime update, the drone transmission rate is 200Hz
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=False  _navdata_demo:=0

# 200Hz real-time update
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=0

# 15Hz real-rime update
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=1
</pre>

### Legacy Navigation Data

Information received from the drone will be published to the `ardrone/navdata` topic. The message type is `ardrone_autonomy::Navdata` and contains the following information:

* `header`: ROS message header
* `batteryPercent`: The remaining charge of the drone's battery (%)
* `state`: The Drone's current state:
        * 0: Unknown
        * 1: Inited
        * 2: Landed
        * 3,7: Flying
        * 4: Hovering
        * 5: Test (?)
        * 6: Taking off
        * 8: Landing
        * 9: Looping (?)
* `rotX`: Left/right tilt in degrees (rotation about the X axis)
* `rotY`: Forward/backward tilt in degrees (rotation about the Y axis)
* `rotZ`: Orientation in degrees (rotation about the Z axis)
* `magX`, `magY`, `magZ`: Magnetometer readings (AR-Drone 2.0 Only) (TBA: Convention)
* `pressure`: Pressure sensed by Drone's barometer (AR-Drone 2.0 Only) (TBA: Unit)
* `temp` : Temperature sensed by Drone's sensor (AR-Drone 2.0 Only) (TBA: Unit)
* `wind_speed`: Estimated wind speed (AR-Drone 2.0 Only) (TBA: Unit)
* `wind_angle`: Estimated wind angle (AR-Drone 2.0 Only) (TBA: Unit)
* `wind_comp_angle`: Estimated wind angle compensation (AR-Drone 2.0 Only) (TBA: Unit)
* `altd`: Estimated altitude (mm)
* `motor1..4`: Motor PWM values
* `vx`, `vy`, `vz`: Linear velocity (mm/s) [TBA: Convention]
* `ax`, `ay`, `az`: Linear acceleration (g) [TBA: Convention]
* `tm`: Timestamp of the data returned by the Drone returned as number of micro-seconds passed since Drone's boot-up.


**NOTE:** The legacy Navdata publishing can be disabled by setting the `enable_legacy_navdata` parameter to `False` (legacy navdata is enabled by default).

### IMU data

The linear acceleration, angular velocity and orientation from the `Navdata` is also published to a standard ROS [`sensor_msgs/Imu`](http://www.ros.org/doc/api/sensor_msgs/html/msg/Imu.html) message. The units are all metric and the reference frame is in `Base` frame. This topic is experimental. The covariance values are specified by specific parameters.

### Magnetometer Data

The normalized magnetometer readings are also published to `ardrone/mag` topic as a standard ROS [`geometry_msgs/Vector3Stamped`](http://www.ros.org/doc/api/geometry_msgs/html/msg/Vector3Stamped.html) message.

### Selective Navdata (Advanced) :new:

You can access almost all sensor readings, debug values and status reports sent from the AR-Drone by using "Selective Navdata". If you set any of following parameters to "True", their corresponding `Navdata` information will be published to a separate topic. For example if you enable `enable_navdata_time`, the driver will publish AR-Drone time information to `ardrone/navdata_time` topic. Most of the names are self-explanatory. Please consult AR-Drone SDK 2.0's documentation (or source code) for more information. All parameters are set to False by default.

<pre>
enable_navdata_trims	        enable_navdata_rc_references 	enable_navdata_pwm	            enable_navdata_altitude	
enable_navdata_vision_raw 	    enable_navdata_vision_of	    enable_navdata_vision	        enable_navdata_vision_perf	
enable_navdata_trackers_send	enable_navdata_vision_detect	enable_navdata_watchdog	        enable_navdata_adc_data_frame	
enable_navdata_video_stream	    enable_navdata_games	        enable_navdata_pressure_raw	    enable_navdata_magneto	
enable_navdata_wind_speed	    enable_navdata_kalman_pressure	enable_navdata_hdvideo_stream	enable_navdata_wifi	enable_navdata_zimmu_3000	
</pre>

**HINT:** You can `rostopic type ardrone/navdata_time | rosmsg show` command for each topic to inspect its published message's data structure.

### Cameras

Both AR-Drone 1.0 and 2.0 are equipped with two cameras. One frontal camera pointing forward and one vertical camera pointing downward. This driver will create three topics for each drone: `ardrone/image_raw`, `ardrone/front/image_raw` and `ardrone/bottom/image_raw`. Each of these three are standard [ROS camera interface](http://ros.org/wiki/camera_drivers) and publish messages of type [image transport](http://www.ros.org/wiki/image_transport). The driver is also a standard [ROS camera driver](http://www.ros.org/wiki/camera_drivers), therefor if camera calibration information is provided either as a set of ROS parameters or appropriate `ardrone_front.yaml` and/or `ardrone_bottom.yaml`, the information will be published in appropriate `camera_info` topics. Please check the FAQ section for more information.

* The `ardrone/*` will always contain the selected camera's video stream and information.

* The way that the other two streams work depend on the type of Drone.

    * Drone 1

    Drone 1 supports four modes of video streams: Front camera only, bottom camera only, front camera with bottom camera inside (picture in picture) and bottom camera with front camera inside (picture in picture). According to active configuration mode, the driver decomposes the PIP stream and publishes pure front/bottom streams to corresponding topics. The `camera_info` topic will include the correct image size.

    * Drone 2

    Drone 2 does not support PIP feature anymore, therefore only one of `ardrone/front` or `ardrone/bottom` topics will be updated based on which camera is selected at the time.

### Tag Detection

The `Navdata` message also returns the special tags that are detected by the Drone's on-board vision processing system. To learn more about the system and the way it works please consult AR-Drone SDK 2.0's [developers guide](https://projects.ardrone.org/projects/show/ardrone-api/). These tags are being detected on both drone's video cameras on-board at 30fps. To configure (or disable) this feature look at the "Parameters" section in this documentation.

The detected tags' type and position in Drone's camera frame will be published to the following variables in `Navdata` message:

* `tags_count`: The number of detected tags.
* `tags_type[]`: Vector of types of detected tags (details below)
* `tags_xc[]`, `tags_yc[]`, `tags_width[]`, `tags_height[]`: Vector of position components and size components for each tag. These numbers are expressed in numbers between [0,1000]. You need to convert them back to pixel unit using the corresponding camera's resolution (can be obtained front `camera_info` topic).
* `tags_orientation[]`: For the tags that support orientation, this is the vector that contains the tag orientation expressed in degrees [0..360).

By default, the driver will configure the drone to look for _oriented roundels_ using bottom camera and _2D tags v2_ on indoor shells (_orange-yellow_) using front camera. For information on how to extract information from `tags_type` field. Check the FAQ section in the end.

### Update Frequencies

TBA.

## Sending Commands to AR-Drone

The drone will *takeoff*, *land* or *emergency stop/reset* by publishing an `Empty` ROS messages to the following topics: `ardrone/takeoff`, `ardrone/land` and `ardrone/reset` respectively.

In order to fly the drone after takeoff, you can publish a message of type [`geometry_msgs::Twist`](http://www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html) to the `cmd_vel` topic.

        -linear.x: move backward
        +linear.x: move forward
        -linear.y: move right
        +linear.y: move left
        -linear.z: move down
        +linear.z: move up

        -angular.z: turn left
        +angular.z: turn right

The range for each component should be between -1.0 and 1.0. The maximum range can be configured using ROS parameters discussed later in this document. 

### Hover Modes :new:

`geometry_msgs::Twist` has two other member variable called `angular.x` and `angular.y` which can be used to enable/disable "auto-hover" mode. "auto-hover" is enabled when all six components are set to **zero**. If you want the drone not to enter "auto hover" mode in cases you set the first four components to zero, set `angular.x` and `angular.y` to arbitrary **non-zero** values.

## Coordinate Frames

The driver publishes two [`tf`](http://www.ros.org/wiki/tf) transforms between three reference frames: `${tf_prefix}/${base_prefix}_link`, `${tf_prefix}/${base_prefix}_frontcam` and `${tf_prefix}/${base_prefix}_bottomcam`. The `${tf_prefix}` is ROS standard way to handle multi-robot `tf` trees and can be set using `tf_prefix` parameters, by default it is empty. The `${base_link}` is the shared name prefix of all three reference frames and can also be set using parameters, by default it has the value of `ardrone_base`. Using default parameters, the three frames would be: `ardrone_base_link`, `ardrone_base_frontcam` and `ardrone_base_bottomcam`. By default the root frame is  `ardrone_base_link`. Therefor `ardrone_base_frontcam` and `ardrone_base_bottomcam` are children of `ardrone_base_link` in the published `tf` tree. This can be changed using `root_frame` parameter.

The `frame_id` field in header of all published topics (navdata, imu, cameras) will have the appropriate frame names. All frames are [ROS REP 103](http://www.ros.org/reps/rep-0103.html) compatible.

## Services

### Toggle AR-Drone's Camera

Calling `ardrone/togglecam` service with no parameters will change the active video camera stream. (e.g `rosservice call /ardrone/togglecam`).

`ardrone/setcamchannel` service directly sets the current active camera channel. One parameter (`uint8 channel
`) should be sent to this service. For AR-Drone 1.0 the valid values are [0..3] and for AR-Drone 2.0 the valid values are [0..1]. The order is similar to the order described in "Cameras" section.

### LED Animations

Calling `ardrone/setledanimation` service will invoke one of 14 pre-defined LED animations for the drone. The parameters are

* `uint8 type`: The type of animation which is a number in range [0..13]
* `float32 freq`: The frequency of the animation in Hz
* `uint8 duration`: The duration of the animation in Seconds.

The `type` parameter will map [in order] to one of these animations (check `srv/LedAnim.srv` for more details):

        BLINK_GREEN_RED, BLINK_GREEN, BLINK_RED, BLINK_ORANGE,
        SNAKE_GREEN_RED, FIRE, STANDARD, RED, GREEN, RED_SNAKE,BLANK,
        LEFT_GREEN_RIGHT_RED, LEFT_RED_RIGHT_GREEN, BLINK_STANDARD`

You can test these animations in command line using commands like `rosservice call /ardrone/setledanimation 1 4 5`

### Flight Animations :new:

Calling `ardrone/setflightanimation` service will execute one of 20 pre-defined flight animations for the drone. The parameters are:

* `uint8 type`: The type of flight animation, a number in range [0..19]
* `uint16 duration`: The duration of the animation. Use 0 for default duration (recommended)

The `type` parameter will map [in order] to one of these pre-defined animations (check `srv/FlightAnim.srv` for more details):

    ARDRONE_ANIM_PHI_M30_DEG, ARDRONE_ANIM_PHI_30_DEG, ARDRONE_ANIM_THETA_M30_DEG, ARDRONE_ANIM_THETA_30_DEG,
    ARDRONE_ANIM_THETA_20DEG_YAW_200DEG, ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG, ARDRONE_ANIM_TURNAROUND,
    ARDRONE_ANIM_TURNAROUND_GODOWN, ARDRONE_ANIM_YAW_SHAKE, ARDRONE_ANIM_YAW_DANCE, ARDRONE_ANIM_PHI_DANCE,
    ARDRONE_ANIM_THETA_DANCE, ARDRONE_ANIM_VZ_DANCE, ARDRONE_ANIM_WAVE, ARDRONE_ANIM_PHI_THETA_MIXED,
    ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED, ARDRONE_ANIM_FLIP_AHEAD, ARDRONE_ANIM_FLIP_BEHIND, ARDRONE_ANIM_FLIP_LEFT,
    ARDRONE_ANIM_FLIP_RIGHT

You can test these animations in command line using commands like `rosservice call /ardrone/setflightanimation 1 0` while drone is flying.

Please be extra cautious about using animations, especially flip animations.

### IMU Calibration

If `do_imu_caliberation` parameter is set to true, calling `ardrone/imu_recalib` service will make the driver recalculate the biases in IMU data based on data from a short sampling period.

### Flat Trim

Calling `ardrone/flattrim` service without any parameter will send a "Flat Trim" request to AR-Drone to re-calibrate its rotation estimates assuming that it is on a flat surface. Do not call this service while Drone is flying or while the drone is not actually on a flat surface.

### Record to USB Stick

Calling `ardrone/setrecord` service will enable and disable recording to the USB stick. The service takes a simple `1` to enable or `0` to disable. So you can turn on recording to the USB stick with `rosservice call /ardrone/setrecord 1`

## Parameters

### AR-Drone Specific Parameters

The parameters listed below are named according to AR-Drone's SDK 2.0 configuration. Unless you set the parameters using `rosparam` or in your `launch` file, the default values will be used. These values are applied during driver's initialization phase. Please refer to AR-Drone SDK 2.0's [developer's guide](https://projects.ardrone.org/projects/show/ardrone-api/) for information about valid values. Not all the parameters will be needed during regular usage of the AR-Drone, please consult the example launch file `launch/ardrone.launch` for frequent parameters.

    altitude, altitude_max, altitude_min, ardrone_name, autonomous_flight, bitrate, bitrate_ctrl_mode, 
    bitrate_storage, codec_fps, com_watchdog, control_iphone_tilt, control_level, control_vz_max, 
    control_yaw, detect_type, detections_select_h, detections_select_v, detections_select_v_hsync, 
    enemy_colors, enemy_without_shell, euler_angle_max, flight_anim, flight_without_shell, flying_mode, 
    groundstripe_colors, hovering_range, indoor_control_vz_max, indoor_control_yaw, indoor_euler_angle_max, 
    latitude, leds_anim, longitude, manual_trim, max_bitrate, max_size, navdata_demo, navdata_options, 
    nb_files, outdoor, outdoor_control_vz_max, outdoor_control_yaw, outdoor_euler_angle_max, output, 
    owner_mac, ssid_multi_player, ssid_single_player, travelling_enable, travelling_mode, ultrasound_freq, 
    ultrasound_watchdog, userbox_cmd, video_channel, video_codec, video_enable, video_file_index, 
    video_live_socket, video_on_usb, video_slices, vision_enable, wifi_mode, wifi_rate

[This wiki page](https://github.com/AutonomyLab/ardrone_autonomy/wiki/AR-Drone-Parameters) includes more information about each of above parameters.
 
### Other Parameters

These parameters control the behaviour of the driver.

* `drone_frame_id` - The "frame_id" prefix to be used in all `tf` frame names - default: "ardrone_base"
* `root_frame` - The default root in drone's `tf` tree (0: _link, 1: _frontcam, 2: _bottomcam) - Default: 0
* `cov/imu_la`, `cov/imu_av` & `cov/imu_or`: List of 9 covariance values to be used in `imu`'s topic linear acceleration, angular velocity and orientation fields respectively - Default: 0.0 for all members (Please check the FAQ section for a sample launch file that shows how to set these values)
* `do_imu_calibration`: [EXPERIMENTAL] Should the drone cancel the biases in IMU data - Default: 0
* `enable_legacy_navdata`: Enable legacy `Navdata` publish - Default: True

## License

The Parrot's license, copyright and disclaimer for `ARDroneLib` are included with the package and can be found in `ParrotLicense.txt` and `ParrotCopyrightAndDisclaimer.txt` files respectively. The other parts of the code are subject to `BSD` license.

## Contributors

[List of all commiters to the repository](http://autonomylab.org/ardrone_autonomy/contribution.html).

- [Mike Hamer](https://github.com/mikehamer) - Added support for proper SDK2 way of configuring the Drone via parameter (critical bug fix) ([More Info](https://github.com/AutonomyLab/ardrone_autonomy/pull/26)). Support for zero-command without hovering ([More info](https://github.com/AutonomyLab/ardrone_autonomy/pull/34)). Full configurable Navdata support ([More info](https://github.com/AutonomyLab/ardrone_autonomy/pull/31)). Support for Real-time navdata and video publishing ([More info](https://github.com/AutonomyLab/ardrone_autonomy/pull/44)). Support for configurable data publishing rate.
- [Jacokb Engel](https://github.com/JakobEngel)
- [Sameer Parekh](https://github.com/sameerparekh) - [Turn on and off USB stick recording](https://github.com/AutonomyLab/ardrone_autonomy/pull/53) - [Seperate Magnetometer Topic](https://github.com/AutonomyLab/ardrone_autonomy/pull/25)
- [Devmax](https://github.com/devmax) - [Flat Trim](https://github.com/AutonomyLab/ardrone_autonomy/issues/18) + Various
comments for enhancements
- [Rachel Brindle](https://github.com/younata) - [Enhanced Navdata for AR-Drone 2.0](https://github.com/AutonomyLab/ardrone_autonomy/pull/2)

## FAQ

### Where should I go next? Is there any ROS package or stack that can be used as a tutorial/sample to use ardrone_autonomy?

Absolutely. Here are some examples:

- [falkor_ardrone](https://github.com/FalkorSystems/falkor_ardrone)

"falkor_ardrone" is a ROS package which uses the "ardrone_autonomy" package to implement autonomous control functionality on an AR.Drone.

- [tum_ardrone](http://www.ros.org/wiki/tum_ardrone)

State Estimation, Autopilot and GUI for ardrone.

- [arl_ardrone_examples](https://github.com/parcon/arl_ardrone_examples)

This ROS stack includes a series of very basic nodes to show users how to develop applications that use the ardrone_autonomy drivers for the AR drone 1.0 and 2.0 quadrotor robot.

- [AR Drone Tutorials](https://github.com/mikehamer/ardrone_tutorials)

This repository contains the source-code for the Up and flying with the AR.Drone and ROS tutorial series, published on [Robohub](http://www.robohub.org).


### How can I report a bug, submit patches or ask for a feature?

`github` offers a nice and convenient issue tracking and social coding platform, it can be used for bug reports and pull/feature request. This is the preferred method. You can also contact the author directly.

If you want to submit a pull request, please submit to `dev-unstable` branch.

### Why the `ARDroneLib` has been patched?

The ARDrone 2.0 SDK has been patched to 1) Enable the lib only build 2) Make its command parsing compatible with ROS and 3) To fix its weird `main()` function issue

### Why the wifi bandwidth usage is too much?

The driver has been configured by default to use the maximum bandwidth allowed to ensure the best quality video stream possible (please take a look at default values in parameters section). That is the reason why the picture quality received from Drone 2.0 using this driver is far better than what you usually get using other software. If for any reason you prefer the lower quality* video stream, change `bitrate_ctrl_mode`, `max_bitrate` and `bitrate` parameters to the default values provided by the AR-Drone developer guide.

(*) Please note that lower quality does not mean lower resolution. By configuring AR-Drone to use bitrate control with limits, the picture gets blurry when there is a movement.

### What is the default configuration for the front camera video stream?

_Drone 1_: 320x240@15fps UVLC Codec
_Drone 2_: 640x360@20fps H264 codec with no record stream

### How can I extract camera information and tag type from `tags_type[]`?

`tag_type` contains information for both source and type of each detected tag. In order to extract information from them you can use the following c macros and enums (taken from `ardrone_api.h`)

```c++
#define DETECTION_EXTRACT_SOURCE(type)  ( ((type)>>16) & 0x0FF )
#define DETECTION_EXTRACT_TAG(type)     ( (type) & 0x0FF )

typedef enum
{
  DETECTION_SOURCE_CAMERA_HORIZONTAL=0,   /*<! Tag was detected on the front camera picture */
  DETECTION_SOURCE_CAMERA_VERTICAL,       /*<! Tag was detected on the vertical camera picture at full speed */
  DETECTION_SOURCE_CAMERA_VERTICAL_HSYNC, /*<! Tag was detected on the vertical camera picture inside the horizontal pipeline */
  DETECTION_SOURCE_CAMERA_NUM,
} DETECTION_SOURCE_CAMERA;

typedef enum
{
  TAG_TYPE_NONE             = 0,
  TAG_TYPE_SHELL_TAG        ,
  TAG_TYPE_ROUNDEL          ,
  TAG_TYPE_ORIENTED_ROUNDEL ,
  TAG_TYPE_STRIPE           ,
  TAG_TYPE_CAP              ,
  TAG_TYPE_SHELL_TAG_V2     ,
  TAG_TYPE_TOWER_SIDE       ,
  TAG_TYPE_BLACK_ROUNDEL    ,
  TAG_TYPE_NUM
} TAG_TYPE;

```

### How can I calibrate the ardrone front/bottom camera?

It is easy to calibrate both cameras using ROS [Camera Calibration](http://www.ros.org/wiki/camera_calibration) package.

First, run the camera_calibration node with appropriate arguments: (For the bottom camera, replace front with bottom)

```bash
rosrun camera_calibration cameracalibrator.py --size [SIZE] --square [SQUARESIZE] image:=/ardrone/front/image_raw camera:=/ardrone/front
```

After successful calibration, press the `commit` button in the UI. The driver will receive the data from the camera calibration node, then will save the information by default in `~/.ros/camera_info/ardrone_front.yaml`. From this point on, whenever you run the driver on the same computer this file will be loaded automatically by the driver and its information will be published to appropriate `camera_info` topic. Sample calibration files for AR-Drone 2.0's cameras are provided in `data/camera_info` folder.

### Can I see a sample ardrone node in a launch file to learn how to set parameters?

Yes, you can check the `launch` folder for sample lanuch file.

### Can I control multiple drones using a single PC? or can I make my drone connect to a wireless router?

With some hacking yes! This [wiki page](https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones) contains some information regarding this issue.

### How to re-compile parrot SDK?

If the `git` updates include a SDK upgrade, you'd better cleanup previous vuild files.

```bash
cd ARDroneLib/Soft/Build
make clean
rm -rf targets_versions
```

Build the SDK and driver again.

## TODO

* Make the `tf` publish optional.
* Add the currently selected camera name to `Navdata`
* [DONE] Add separate topic for drone's debug stream (`navdata_demo`)
* [DONE] Make the `togglecam` service accept parameters
* [DONE] Enrich `Navdata` with magneto meter and baro meter information
