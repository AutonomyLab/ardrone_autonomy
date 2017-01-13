^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ardrone_autonomy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


1.4.1 (2015-10-18)
------------------
* Enable realtime navdata/video in sample launch file
* Fix integer only values for odom.pose.z (fixes `#155 <https://github.com/AutonomyLab/ardrone_autonomy/issues/155>`_) (hattip to @alexjung)
* Use tf_prefix parameter for multiple drone settings (fixes `#156 <https://github.com/AutonomyLab/ardrone_autonomy/issues/156>`_)
  - Use tf_prefix to prefix tf frame names
  - Update single and multiple drone launch files to demo the usage
* Add example launch file for multiple drones.
* Publish TF at the same rate as odometry
* Contributors: Jacob Perron, Mani Monajjemi, Matias N. (v01d)

1.4.0 (2015-04-24)
------------------

* Added support for running multiple instances of the driver on a same machine
* Added support for publishing odometry data and transform
* Deprecated "Setting TF root" and "IMU Calibration"
* Use reception time for video streams 
* Improved documentation. Documentation is now hosted on readthedocs
* Updated `gps-waypoint` branch and its documentation
* Build system improvements + roslint + code cleanups
* Contributors: Mani Monajjemi, v01d, kbogert

1.3.7 (2014-09-08)
------------------
* Use git protocol instead of https for cmake external project (fixes ca-certificate issues on the build farm)

1.3.6 (2014-09-05)
------------------
* Adding git as build dependency to fix binary build

1.3.5 (2014-09-03)
------------------
* Fixed Incorrect orientation in ardrone/imu #113  ht @tomas-c
* Added install rules for launch files. #114 ht @kbogert @fig0451
* ARDroneSDK is now fetched from an external repository. Patches are applied there.
* Add unit for pressure readings to README @garyservin
* Moved header files to include directory #110 @garyservin
* Fix ffmpeg library link order #109 @garyservin
* Contributors: Gary Servin, Mani Monajjemi

1.3.4 (2014-06-03)
------------------
* Decreased the frequency of magnetometer warning message
* Fixed some warnings to address `#104 <https://github.com/AutonomyLab/ardrone_autonomy/issues/104>`_
* Contributors: Mani Monajjemi

1.3.3 (2014-03-01)
------------------
* Fixed tf deprecated setEulerZYX
* New SDK patch to remove deprecated warnings from FFMPEG

1.3.2 (2014-02-25)
------------------
* SDK related bug fixes for building binaries.

1.3.1 (2014-02-18)
------------------
* Preparing the initial binary release.
