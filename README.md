# ardrone_autonomy : A ROS Driver for ARDrone 1.0 & 2.0

## Introduction

"ardrone_autonomy" is a [ROS](http://ros.org/ "Robot Operating System") driver for [Parrot AR-Drone](http://TBA) quadrocopter. This driver is based on official [AR-Drone SDK](http://TBA) version 2.0 and supports both AR-Drone 1.0 and 2.0. "ardrone_autonomy" is a fork of [AR-Drone Brown](http://TBA) driver and being developed in [Autonomy Lab](http://autonomy.cs.sfu.ca) of [Simon Fraser University](http://www.sfu.ca) by [Mani Monajjemi](http://sfu.ca/~mmmonajje). 

## Installation

### Pre-requirements

### Installation Steps

The installation follows the same steps needed usually to compile a ROS driver.

* Get the code: Clone (or download and unpack) the driver to your personal ROS stacks folder (e.g. ~/ros/stacks) and `cd` to it. 

```bash
cd ~/ros/stacks
git clone TBA
rosstack profile && rospack profile
roscd ardrone_autonomy
```

* Compile the AR-Drone SDK: The driver contains a slightly patched version of AR-Drone 2.0 SDK which is located in `ARDroneLib` directory. To compile it, execute the `./build_sdk.sh`. Any system-wide dependencies will be managed by the SDK's build script. You may be asked to install some packages during the installation procedure. You can verify the success of the SDK's build by checking the `lib` folder.

```bash
./build_sdk 
[After a couple of minutes]
ls ./lib
TBA
```

* Compile the driver: You can easily compile the driver by using `rosmake ardrone_autonomy` command.

## How to Run

The driver's executable node is `ardrone_driver`. You can either use `rosrun ardrone_autonomy ardrone_driver` or put it in your launch file.

## Reading from AR-Drone

### Navigation Data

All the drone information will be published to the `ardrone/navdata` topic. The message type is `ardrone_autonomy::Navdata` and contains the following information:

* `header`: ROS message header
* `batteryPercent`: The remaining charge of the Drone (%)
* `state`: The Drone's current state: 
** 0: Unknown
** 1: Inited
** 2: Landed
** 3,7: Flying
** 4: Hovering
** 5: Test (?)
** 6: Taking off
** 8: Landing
** 9: Looping (?)
* `rotx`: left/right tilt in degrees (rotation about the X axis)
* `roty`: forward/backward tilt in degrees (rotation about the Y axis)
* `rotz`: orientation in degrees (rotation about the Z axis)
* `altd`: estimated altitude (cm)
* `vx`, `vy`, `vz`: Linear velocity (mm / s) [TBA: Convention]
* `ax`, `ay`, `az`: Linear acceleration (g) [TBA: Convention]
* `tm`: Timestamp of the data returned by the Drone

### Tag Detection

The `Navdata` message also returns the special tags that are detected by the Drone's on-board vision processing system. To learn more about the system and the way it works please consult AR-Drone SDK 2.0's [developers guide](http://). These tags are being detected on both drone's video cameras on-board at 30fps. To configure (or disable) this feature look at the "Parameters" section below.

[TBA]

### Cameras

Both AR-Drones are equipped with two cameras. One frontal camera pointing forward and one vertical camera pointing downward. This driver will create three topics for each drone: `ardrone/image_raw`, `ardrone/front/image_raw` and `ardrone/bottom/image_raw`. Each of these three are standard [ROS camera interface](http://) and publish [image transport](http://) compatible ROS video streams. 

* The `/ardrone/*` will always contain the selected camera's video stream and information.

* The way that the other two streams work depend on the type of Drone.

** Drone 1

Drone 1 supports four modes of video streams: Front Camera only, Bottom Camera only, Front Camera with bottom camera picture in picture and Bottom camera with front camera picture in picture. According to active configuration mode, the driver decomposes the PIP streams and publishes pure front/bottom streams to corresponding topics. The `camera_info` topic will include the correct image size. 

** Drone 2

Drone 2 does not support PIP feature anymore, therefore only one of `ardrone/front` or `ardrone/bottom` topics will be updated based on which is selected at the time.

## Sending Commands to AR-Drone

The drone will *takeoff*, *land* or *emergency stop/reset* by Publishing an `Empty` ros messages to the following topics: `ardrone/takeoff`, `ardrone/land` and `ardrone/reset`.

In order to fly the drone after takeoff, you can publish a message of type [`geometry_msgs::Twist`](http://TBA) to the `ardrone/cmd_vel` topic. 

[TBA]

The range for each component should be between -1.0 and 1.0. The maximum range can be configured using ROS parameters below. Publishing 0 values for all components will make the drone keep hovering.

## Services

Calling `ardrone/togglecam` service with no parameters will change the active video camera stream.

## Parameters

The parameters listed below are named according to AR-Drone's SDK 2.0 configuration. Unless you set the parameters using `rosparam` or in your `lauch` file, the default values will be used. These values are applied during driver's phase. Please refer to AR-Drone's SDK 2.0 [developer's guide](http://) for information about the valid values.

* `bitrate_ctrl_mode` - default: DISABLED
* `max_bitrate` - (AR-Drone 2.0 only) Default: 4000 Kbps
* `bitrate` -  Default: 4000 Kbps
* `outdoor` - Default: 0
* `flight_without_shell` - Default: 1
* `altitude_max`: - Default: 3000 mm
* `altitude_min`: - Default: 100 mm

## License

## TODO List

* Enrich `Navdata` with magneto meter and baro meter information
* Add separate topic for drone's debug stream (`navdata_demo`)
* Add the currently selected camera name to `Navdata`
* Make the `togglecame` service accept parameter
## FAQ

* How can I report a bug?
* Why the `ARDroneLib` has been patched?

