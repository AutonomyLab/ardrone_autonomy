^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ardrone_autonomy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


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
