============================
Sending Commands to AR-Drone
============================

The drone will `takeoff`, `land` or `emergency stop/reset` if a ROS `std_msgs/Empty <http://docs.ros.org/indigo/api/std_msgs/html/msg/Empty.html>`_ message is published to ``ardrone/takeoff``, ``ardrone/land`` and ``ardrone/reset`` topics respectively.

In order to fly the drone after takeoff, you can publish a message of type `geometry_msgs::Twist <http://www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html>`_ to the ``cmd_vel`` topic::


    -linear.x: move backward
    +linear.x: move forward
    -linear.y: move right
    +linear.y: move left
    -linear.z: move down
    +linear.z: move up

    -angular.z: turn right
    +angular.z: turn left

The range for each component should be between -1.0 and 1.0. The maximum range can be configured using ROS Parameters_ discussed later in this document. 

Hover Modes
-----------

``geometry_msgs::Twist`` has two other member variables ``angular.x`` and ``angular.y`` which can be used to enable/disable "auto-hover" mode. "auto-hover" is enabled when all six components are set to **zero**. If you want the drone not to enter "auto hover" mode in cases you set the first four components to zero, set `angular.x` and `angular.y` to arbitrary **non-zero** values.


