# ardrone_autonomy 

[ROS](http://ros.org) Driver for [Parrot AR-Drone](http://ardrone2.parrot.com/) 1.0 & 2.0 Quadrocopters

* Documentation: http://ardrone-autonomy.readthedocs.org/
* ROS wiki page: http://wiki.ros.org/ardrone_autonomy
* Code API: http://docs.ros.org/indigo/api/ardrone_autonomy/html
* Patched _ARDroneLib_ repository: https://github.com/AutonomyLab/ardronelib
* Author: [Mani Monajjemi](http://mani.im) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca)) + [other contributers](http://ardrone-autonomy.readthedocs.org/en/latest/contributers.html)

## Build Status

* ROS Build farm (_Jade_): [![Build Status](http://build.ros.org/buildStatus/icon?job=Jdev__ardrone_autonomy__ubuntu_trusty_amd64)](http://build.ros.org/job/Jdev__ardrone_autonomy__ubuntu_trusty_amd64/)
* ROS Build farm (_Indigo_): [![Build Status](http://build.ros.org/buildStatus/icon?job=Idev__ardrone_autonomy__ubuntu_trusty_amd64)](http://build.ros.org/job/Idev__ardrone_autonomy__ubuntu_trusty_amd64/)
* Travis (_Jade_/_Indigo_): [![Build Status](https://travis-ci.org/AutonomyLab/ardrone_autonomy.svg?branch=indigo-devel)](https://travis-ci.org/AutonomyLab/ardrone_autonomy)

## New
* There are some changes in the code of ardrone_driver to use the front camera to control the flight instead of the bottom camera. For this purpose, the front camera has been moved pointing down. The position of the new placement of the camera has been updated in the ardrone_driver.cpp in reference to the ardrone coordinate frame.
