============
Installation
============

Binary install
==============

On supported *Ubuntu* platform and for ROS *Indigo*, *Hydro* and *Groovy* you can install the driver by running ``apt-get install ros-*-ardrone-autonomy`` e.g. ``apt-get install ros-hydro-ardrone-autonomy`` in a terminal.

Compile from source
===============================

The bundled AR-Drone SDK has its own build system which usually handles system wide dependencies itself. The ROS package depends on these standard ROS packages: ``roscpp``, ``image_transport``, ``sensor_msgs``, ``tf``, ``camera_info_manager``, `nav_msgs` and ``std_srvs``.

The installation follows the same steps needed usually to compile a ROS driver using `catkin <http://wiki.ros.org/catkin>`. Clone (or download and unpack) the driver to the ``src`` folder of a new or existing catkin `workspace <http://wiki.ros.org/catkin/Tutorials/create_a_workspace>` (e.g ``~/catkin_ws/src``), then run ``catkin_make`` to compile it. Assuming you are compiling for ROS *Indigo*:

.. code-block:: bash

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel
    $ cd ~/catkin_ws
    $ rosdep install --from-paths src -i
    $ catkin_make

