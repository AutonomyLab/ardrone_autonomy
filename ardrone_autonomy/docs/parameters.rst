==========
Parameters
==========

AR-Drone Specific Parameters
----------------------------

The parameters listed below are named according to AR-Drone's SDK 2.0 configuration. Unless you set the parameters using ``rosparam`` or in your ``launch`` file, the default values will be used. These values are applied during driver's initialization phase. Please refer to AR-Drone SDK 2.0's `developer's guide <https://projects.ardrone.org/projects/show/ardrone-api/>`_ for information about accepted values. Not all the parameters are needed during regular usage of the AR-Drone, please consult the example launch file ``launch/ardrone.launch`` for frequently used ones::

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

`This wiki page <https://github.com/AutonomyLab/ardrone_autonomy/wiki/AR-Drone-Parameters>`_ includes more information about each of above parameters.
 
Other Parameters
----------------

These parameters control the behavior of the driver.

* ``drone_frame_id`` - The "frame_id" prefix to be used in all `tf` frame names - default: `ardrone_base`
* ``cov/imu_la``, ``cov/imu_av`` and ``cov/imu_or``: List of 9 covariance values to be used to fill `imu`'s topic linear acceleration, angular velocity and orientation fields respectively - Default: 0.0 for all members (Please check the :doc:`FAQ` section for a sample launch file that shows how to set these values)
* ``enable_legacy_navdata``: Enables :ref:`legacynavdata` publishing - Default: True
