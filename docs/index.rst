
================
ardrone_autonomy 
================

`ardrone_autonomy` is a `ROS <http://ros.org>`_ driver for `Parrot AR-Drone <http://ardrone2.parrot.com/>`_ 1.0 & 2.0 quadrocopter. This driver is based on official `AR-Drone SDK <https://projects.ardrone.org/>`_ version 2.0.1. `ardrone_autonomy` is a fork of `AR-Drone Brown <http://code.google.com/p/brown-ros-pkg/wiki/ardrone_brown>`_ driver. This package is developed in `Autonomy Lab <http://autonomylab.org>`_ of `Simon Fraser University <http://www.sfu.ca>`_ by `Mani Monajjemi <http://mani.im>`_  and other :doc:`./contributers` .

External Links: `Source code and issue tracker <https://github.com/AutonomyLab/ardrone_autonomy>`_ | `ROS wiki page <http://wiki.ros.org/ardrone_autonomy>`_ | `Code API <http://docs.ros.org/indigo/api/ardrone_autonomy/html/>`_ 


Updates
-------

- **April 2014**: 1.4
  
  - Publish Odometry (`#123 <https://github.com/AutonomyLab/ardrone_autonomy/pull/123>`_)
  - Support for multiple instances of the driver on a single machine (`#98 <https://github.com/AutonomyLab/ardrone_autonomy/pull/98>`_ and `ardronelib/#2 <https://github.com/AutonomyLab/ardronelib/pull/2>`_)
  - Use reception time for video streams (`#89 <https://github.com/AutonomyLab/ardrone_autonomy/pull/89>`_)
  - Refactoring of source code and build system
  - Deprecated setting TF root frame (`6afa19 <https://github.com/AutonomyLab/ardrone_autonomy/commit/6afa19729b4faf03ac92b8f772c8ad7a2c48728e>`_)
  - Deprecated auto IMU calibration (`6afa19 <https://github.com/AutonomyLab/ardrone_autonomy/commit/6afa19729b4faf03ac92b8f772c8ad7a2c48728e>`_)

- **September 3 2014** : 1.3.5: `Bug Fixes & Minor Improvements <https://github.com/AutonomyLab/ ardrone_autonomy/milestones/1.3.5>`_

- **March 14 2014**: The binary packages of the driver are now built on `ROS build farm <http://wiki.ros.org/BuildFarm>`_. You can install the driver for ROS *Indigo*, *Hydro* and *Groovy* using ``apt-get`` on *Ubuntu*.

- **January 17 2014**:

  - Fully *catkinized* package (`#75 <https://github.com/AutonomyLab/ardrone_autonomy/pull/75>`_ & `#79 <https://github.com/AutonomyLab/ardrone_autonomy/pull/79>`_).
  - ARDroneLib has been configured to be built as an external project. ARDroneLib is replaced by the vanilla SDK's stripped tarball. (`More info <https://github.com/AutonomyLab/ardrone_autonomy/pull/80>`_).

- **October 22 2013**: Update to Parrot SDK 2.0.1 (Fixes crashes on 2.4.x firmwares, no support for flight recorder (yet).

- **February 13 2013**: Support for USB key recording (`More info <https://github.com/AutonomyLab/ardrone_autonomy/pull/53>`_). Motor PWM added to legacy Navdata.

- **January 9 2013**: 
  
  - ROS Groovy support.
  - Support for zero-command without hovering (`More info <https://github.com/AutonomyLab/ardrone_autonomy/pull/34>`_). 
  - Fully configurable Navdata support (`More info <https://github.com/AutonomyLab/ardrone_autonomy/pull/31>`_). 
  - Support for :ref:`flightanimation`. 
  - Support for Real-time navdata and video publishing (`More info <https://github.com/AutonomyLab/ardrone_autonomy/pull/44>`_).
  - Support for configurable data publishing rate.

- **November 9 2012**: Critical Bug in sending configurations to drone fixed and more parameters are supported (`More info <https://github.com/AutonomyLab/ardrone_autonomy/issues/24>`_). Separate topic for magnetometer data added (`More info <https://github.com/AutonomyLab/ardrone_autonomy/pull/25>`_).

- **September 5 2012**: Experimental automatic IMU bias removal.

- **August 27 2012**: Thread-safe SDK data access. Synchronized `navdata` and `camera` topics.

- **August 20 2012**: The driver is now provides ROS standard camera interface.

- **August 17 2012**: Experimental ``tf`` support added. New published topic ``imu``.

- **August 1 2012**: Enhanced `Navdata` message. `Navdata` now includes magnetometer data, barometer data, temperature and wind information for AR-Drone 2. (`Issue #2 <https://github.com/AutonomyLab/ardrone_autonomy/pull/2>`_)

- **July 27 2012**: :ref:`ledanimation` Support added to the driver as a service

- **July 19 2012**: Initial Public Release

Table of Contents
-----------------

.. toctree::
    :maxdepth: 2
    
    installation
    usage
    reading
    frames
    commands    
    services
    parameters
    license
    contributers
    FAQ
