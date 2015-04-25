===
FAQ 
===

Where should I go next? Is there any ROS package or stack that can be used as a tutorial/sample to use ardrone_autonomy?
-------------------------------------------------------------------------------------------------------------------------
Absolutely. Here are some examples:

- `falkor_ardrone <https://github.com/FalkorSystems/falkor_ardrone>`_

"falkor_ardrone" is a ROS package which uses the "ardrone_autonomy" package to implement autonomous control functionality on an AR.Drone.

- `tum_ardrone <http://www.ros.org/wiki/tum_ardrone>`_

State Estimation, Autopilot and GUI for ardrone.

- `arl_ardrone_examples <https://github.com/parcon/arl_ardrone_examples>`_

This ROS stack includes a series of very basic nodes to show users how to develop applications that use the ardrone_autonomy drivers for the AR drone 1.0 and 2.0 quadrotor robot.

- `AR Drone Tutorials <https://github.com/mikehamer/ardrone_tutorials>`_

This repository contains the source-code for the Up and flying with the AR.Drone and ROS tutorial series, published on [Robohub](http://www.robohub.org).

- `tum_simulator <http://wiki.ros.org/tum_simulator>`_

AR Drone simulation in `Gazebo <http://wiki.ros.org/gazebo_ros_pkgs>`_, compatible with `ardrone_autonomy`.

How can I report a bug, submit patches or ask for a feature?
------------------------------------------------------------

`github` offers a nice and convenient issue tracking and social coding platform, it can be used for bug reports and pull/feature request. This is the preferred method. You can also contact the author directly.

Why the `ARDroneLib` has been patched?
--------------------------------------

The ARDrone 2.0.1 SDK has been patched to 1) Enable the lib only build 2) Make its command line parsing compatible with ROS and 3) To fix its weird `main()` function issue. The patched SDK is being hosted on an `external repository <https://github.com/AutonomyLab/ardronelib>`_.

Why the wifi bandwidth usage is too much?
-----------------------------------------

The driver has been configured by default to use the maximum bandwidth allowed to ensure the best quality video stream possible (please take a look at default values in parameters section). That is why the picture quality received from Drone 2.0 using this driver is far better than what you usually get using other software. If for any reason you prefer the lower quality* video stream, change `bitrate_ctrl_mode`, `max_bitrate` and `bitrate` parameters to the default values mentioned in the AR-Drone developer guide.

.. note::
  Please note that lower quality does not mean lower resolution. By configuring AR-Drone to use bitrate control with limits, the picture gets blurry when there is a movement.

What is the default configuration for the front camera video stream?
---------------------------------------------------------------------

*Drone 1*: `320x240@15fps UVLC Codec`
*Drone 2*: `640x360@20fps H264 codec with no record stream`

How can I extract camera information and tag type from `tags_type[]`?
---------------------------------------------------------------------

`tag_type` contains information for both source and type of each detected tag. In order to extract information from them you can use the following c macros and enums (taken from `ardrone_api.h`)

.. code-block:: c

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


How can I calibrate the ardrone front/bottom camera?
----------------------------------------------------

It is easy to calibrate both cameras using ROS `Camera Calibration <http://www.ros.org/wiki/camera_calibration) package>`_.

First, run the camera_calibration node with appropriate arguments: (For the bottom camera, replace front with bottom)

.. code-block:: bash

  rosrun camera_calibration cameracalibrator.py --size [SIZE] --square [SQUARESIZE] image:=/ardrone/front/image_raw camera:=/ardrone/front

After successful calibration, press the `commit` button in the UI. The driver will receive the data from the camera calibration node, then will save the information by default in ``~/.ros/camera_info/ardrone_front.yaml``. From this point on, whenever you run the driver on the same computer this file will be loaded automatically by the driver and its information will be published to appropriate `camera_info` topic. Sample calibration files for AR-Drone 2.0's cameras are provided in ``data/camera_info`` folder.

Can I control multiple drones using a single PC? or can I make my drone connect to a wireless router?
------------------------------------------------------------------------------------------------------------

Since version 1.4, the driver supports connecting to multiple AR-Drones from a single PC. Thanks to efforts and patches provided by @kbogert. For more information please check this `wiki page <https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones>`_.

Is there any support for GPS (Parrot Flight Recorder)
-----------------------------------------------------

Yes but it is experimental. The code is maintained in a separate branch (`gps-waypoint <https://github.com/AutonomyLab/ardrone_autonomy/tree/gps-waypoint>`_). For more information see `this documentation <http://ardrone-autonomy.readthedocs.org/en/gps-waypoint/>`_.
