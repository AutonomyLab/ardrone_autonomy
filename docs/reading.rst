=====================
Reading from AR-Drone
=====================

Update frequencies
------------------

**Drone Update Frequencies**: The drone's data transmission update frequency depends on `navdata_demo` parameter. When it is set to `1`, the transmission frequency is set `15Hz`, otherwise transmission frequency is set to `200Hz`. (`navdata_demo` is a numeric parameter not Boolean, so use 1 and 0 (not True/False) to set/unset it)

**Driver Update Frequencies**: The driver can operate in two modes: real-time or fixed rate. When the `realtime_navdata` parameter is set to True, the driver publishes any received information instantly. When it is set to False, the driver caches the received data first, then sends them at a fixed rate. This rate is configured via `looprate` parameter. The default configuration is: ``realtime_navdata=False`` and ``looprate=50``. 

Please note that if `looprate` is smaller than the drone's transmission frequency, some data is going to be lost. The driver's start-up output shows the current configuration. You can also use `rostopic hz` command to check the publish rate of the driver.


.. code-block:: bash

    # Default Setting - 50Hz non-realtime update, the drone transmission rate is 200Hz
    $ rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=False  _navdata_demo:=0

    # 200Hz real-time update
    $ rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=0

    # 15Hz real-rime update
    $ rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=1

.. _legacynavdata:

Legacy navigation data
----------------------

Information received from the drone is published to the ``ardrone/navdata`` topic. The message type is ``ardrone_autonomy::Navdata`` and contains the following information: (`Full specifications <http://docs.ros.org/indigo/api/ardrone_autonomy/html/msg/Navdata.html>`_)

* ``header``: ROS message header
* ``batteryPercent``: The remaining charge of the drone's battery (%)
* ``state``: The Drone's current state:
        * 0: Unknown
        * 1: Inited
        * 2: Landed
        * 3,7: Flying
        * 4: Hovering
        * 5: Test (?)
        * 6: Taking off
        * 8: Landing
        * 9: Looping (?)
* ``rotX``: Left/right tilt in degrees (rotation about the X axis)
* ``rotY``: Forward/backward tilt in degrees (rotation about the Y axis)
* ``rotZ``: Orientation in degrees (rotation about the Z axis)
* ``magX``, `magY`, `magZ`: Magnetometer readings (AR-Drone 2.0 Only) (TBA: Convention)
* ``pressure``: Pressure sensed by Drone's barometer (AR-Drone 2.0 Only) (Pa)
* ``temp`` : Temperature sensed by Drone's sensor (AR-Drone 2.0 Only) (TBA: Unit)
* ``wind_speed``: Estimated wind speed (AR-Drone 2.0 Only) (TBA: Unit)
* ``wind_angle``: Estimated wind angle (AR-Drone 2.0 Only) (TBA: Unit)
* ``wind_comp_angle``: Estimated wind angle compensation (AR-Drone 2.0 Only) (TBA: Unit)
* ``altd``: Estimated altitude (mm)
* ``motor1..4``: Motor PWM values
* ``vx``, ``vy``, ``vz``: Linear velocity (mm/s) [TBA: Convention]
* ``ax``, ``ay``, ``az``: Linear acceleration (g) [TBA: Convention]
* ``tm``: Timestamp of the data returned by the Drone returned as number of micro-seconds passed since Drone's boot-up.

.. note::

    The legacy Navdata publishing can be disabled by setting the `enable_legacy_navdata` parameter to `False` (legacy navdata is enabled by default).

IMU data
--------

Linear acceleration, angular velocity and orientation of the drone is published to a standard ROS `sensor_msgs/Imu <http://www.ros.org/doc/api/sensor_msgs/html/msg/Imu.html>`_ message. The units are all metric and `TF` reference frame is set to drone's `base` frame. The covariance values are specified through ``cov/imu_la``, ``cov/imu_av`` and ``cov/imu_or`` parameters. For More information, please check the :doc:`parameters` section.

Magnetometer data
-----------------

The normalized magnetometer readings are published to ``ardrone/mag`` topic as a standard ROS `geometry_msgs/Vector3Stamped <http://www.ros.org/doc/api/geometry_msgs/html/msg/Vector3Stamped.html>`_ message.

Odometry data
-------------

.. versionadded:: 1.4

The driver calculates and publishes Odometry data by integrating velocity estimates reported by the drone (which is based on optical flow). The data is published as `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_ messages to ``ardrone/odometry`` topic. The corresponding `TF` transform is also published as `odom -> base` transformation.

Selective Navdata (advanced)
----------------------------

You can access almost all sensor readings, debug values and status reports sent from the AR-Drone by using `Selective Navdata`. If you set any of following parameters to `True`, their corresponding `Navdata` information will be published to a separate topic. For example if you enable ``enable_navdata_time``, the driver will publish AR-Drone time information to ``ardrone/navdata_time`` topic. Most of the names are self-explanatory. Please consult AR-Drone SDK 2.0's documentation (or source code) for more information. All parameters are set to False by default.

.. code-block:: text

    enable_navdata_trims            enable_navdata_rc_references    enable_navdata_pwm              enable_navdata_altitude 
    enable_navdata_vision_raw       enable_navdata_vision_of        enable_navdata_vision           enable_navdata_vision_perf  
    enable_navdata_trackers_send    enable_navdata_vision_detect    enable_navdata_watchdog         enable_navdata_adc_data_frame   
    enable_navdata_video_stream     enable_navdata_games            enable_navdata_pressure_raw     enable_navdata_magneto  
    enable_navdata_wind_speed       enable_navdata_kalman_pressure  enable_navdata_hdvideo_stream   enable_navdata_wifi enable_navdata_zimmu_3000   

.. note::
    
    You can use ``rostopic type ardrone/navdata_time | rosmsg show`` command for each topic to inspect its published message's data structure.

.. _cameras:

Cameras
-------

Both AR-Drone 1.0 and 2.0 are equipped with two cameras. One frontal camera pointing forward and one vertical camera pointing downward. This driver will create three topics for each drone: ``ardrone/image_ra``, ``ardrone/front/image_raw`` and ``ardrone/bottom/image_raw``. Each of these three are standard `ROS camera interface <http://ros.org/wiki/camera_drivers>`_ and publish messages of type `image transport <http://www.ros.org/wiki/image_transport>`_. The driver is also a standard `ROS camera driver <http://www.ros.org/wiki/camera_drivers>`_, therefor if camera calibration information is provided either as a set of ROS parameters or through ``ardrone_front.yaml`` and/or ``ardrone_bottom.yaml`` files, calibration information will be also published via `camera_info` topics. Please check the :doc:`FAQ` section for more information.

* The `ardrone/*` will always contain the selected camera's video stream and information.

The way that the other two streams work depend on the type of Drone.

AR-Drone 1
^^^^^^^^^^

AR-Drone 1 supports four modes of video streams: Front camera only, bottom camera only, front camera with bottom camera inside (picture in picture) and bottom camera with front camera inside (picture in picture). According to active configuration mode, the driver decomposes the PIP stream and publishes pure front/bottom streams to corresponding topics. The `camera_info` topic will include the correct image size.

AR-Drone 2
^^^^^^^^^^

AR-Drone 2 does not support PIP feature anymore, therefore only one of `ardrone/front` or `ardrone/bottom` topics will be updated based on which camera is selected at the time.

Tag detection
-------------

The ``Navdata`` message also contains information about the special tags that are detected by the drone's on-board vision processing system. To learn more about the system and the way it works please consult AR-Drone SDK 2.0's `developers guide <https://projects.ardrone.org/projects/show/ardrone-api/>`_. These tags are detected on both video cameras on-board at `30fps`. To configure (or disable) this feature check the :doc:`parameters` section.

Information about these detected tags are published through the following field of the `Legacy Navigation data`_ message.

* ``tags_count``: The number of detected tags.
* ``tags_type[]``: Vector of types of detected tags (details below)
* ``tags_xc[]``, ``tags_yc[]``, ``tags_width[]``, ``tags_height[]``: Vector of position components and size components for each tag. These numbers are expressed in numbers between [0,1000]. You need to convert them back to pixel unit using the corresponding camera's resolution (can be obtained front `camera_info` topic).
* ``tags_orientation[]``: For the tags that support orientation, this is the vector that contains the tag orientation expressed in degrees [0..360).

By default, the driver configures the drone to look for `oriented roundels` using bottom camera and `2D tags v2` on indoor shells (`orange-yellow`) using front camera. For information on how to extract information from `tags_type` field. Check the :doc:`FAQ` section in the end.
