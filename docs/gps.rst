===========
GPS Support
===========

.. warning::
    This is an extremely experimental feature of the driver that requires a `Parrot flight recorder peripheral <http://ardrone2.parrot.com/apps/flight-recorder/>`_. This feature is only available if you compile the `gps-waypoint <https://github.com/AutonomyLab/ardrone_autonomy/tree/gps-waypoint>`_ branch of the driver from source. Use at your own risk.

.. note::
    The `gps-waypoint` branch tracks the latest development branch of the driver (currently `indigo-devel`).

Compiling `gps-waypoint` branch from source
-------------------------------------------

.. code-block:: bash

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b gps-waypoint
    $ cd ~/catkin_ws
    $ rosdep install --from-paths src -i
    $ catkin_make

Reading GPS Data
----------------

First run the driver with GPS support by using :ref:`selectivenavdata` :

.. code-block:: bash

    $ rosrun ardrone_autonomy ardrone_driver _enable_navdata_gps:=True

GPS information is published to ``ardrone/navdata_gps``. The message is of custom type `ardrone_autonomy/navdata_gps`::

    Header  header
    uint16 tag
    uint16 size
    float64 latitude
    float64 longitude
    float64 elevation
    float64 hdop
    uint32   data_available
    bool zero_validated 
    bool wpt_validated 
    float64 lat0 
    float64 long0 
    float64 lat_fused 
    float64 long_fused 
    uint32 gps_state 
    float32 X_traj 
    float32 X_ref 
    float32 Y_traj 
    float32 Y_ref 
    float32 theta_p 
    float32 phi_p 
    float32 theta_i 
    float32 phi_i 
    float32 theta_d 
    float32 phi_d 
    float64 vdop
    float64 pdop
    float32 speed
    uint32  lastFrameTimestamp
    float32 degree
    float32 degree_magnetic
    float32 ehpe 
    float32 ehve 
    float32 c_n0  # Signal to noise ratio (average of the four best satellites)
    uint32  nbsat # Number of acquired satellites
    navdata_gps_channel[12] channels
    bool is_gps_plugged
    uint32 ephemerisStatus
    float32 vx_traj 
    float32 vy_traj 
    uint32 firmwareStatus

GPS waypoint navigation
-----------------------

.. note::
    Parrot's official SDK does not expose this functionality. This feature was added to the driver by reverse engineering the protocol used by `FreeFlight App`. All GPS related patches to the SDK are available `here <https://github.com/AutonomyLab/ardronelib/tree/gps>`_.

AR-Drone 2.0 with Flight Recorder can perform on-board GPS waypoint navigation. Currently, the driver supports sending one target waypoint to the drone through ``ardrone/setgpstarget`` service. To set/overwrite the target, call this service with a request of type `geographic_msgs/WayPoint <http://docs.ros.org/indigo/api/geographic_msgs/html/msg/WayPoint.html>`_. ``position`` field determines `latitude`, `longitude` and `altitude` of the the target (`more info <http://docs.ros.org/indigo/api/geographic_msgs/html/msg/GeoPoint.html>`_). ``props`` field should include a vector of ROS `key-value <http://docs.ros.org/indigo/api/geographic_msgs/html/msg/KeyValue.html>`_ pairs. Supported `keys` are:

* ``velocity``: `value` sets the desired linear velocity in `m/s`
* ``orientation``: `value` sets the desired orientation at target in radians

After setting the target position, you need to enable autonomous flight mode by calling ``ardrone/setautoflight`` with ``enable = True`` (autonomous flight mode is turned off by default). The service is of type `ardrone_autonomy/RecordEnable <http://docs.ros.org/indigo/api/ardrone_autonomy/html/srv/RecordEnable.html>`_. Similary you can disable the autonomous flight mode by calling this service with ``enable = False``.

.. note::
    TODO: Add examples (e.g. CLI or code samples)

Contributors to GPS patches
---------------------------

- `Mani Monajjemi <http://mani.im>`_
- `Gary Servin <https://github.com/garyservin>`_
