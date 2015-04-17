========
Services
========

.. note::
    You can find the API documentation of all services `here <http://docs.ros.org/indigo/api/ardrone_autonomy/html/index-msg.html>`_.


Toggle Camera
------------------------

Calling ``ardrone/togglecam`` service with no parameters will change the active video camera stream. (e.g ``rosservice call /ardrone/togglecam``).

``ardrone/setcamchannel`` service directly sets the current active camera channel. One parameter (``uint8 channel
``) must be set when calling this service. For AR-Drone 1.0, valid values are `{0,1,2,3}` while for AR-Drone 2.0 these values are `{0,1}`. The order is similar to the order described in :ref:`cameras` section.

.. _ledanimation:

LED Animations
--------------

Calling ``ardrone/setledanimation`` service invokes one of 14 pre-defined LED animations for the drone. The parameters are

* ``uint8 type``: The type of animation which is a number in range [0..13]
* ``float32 freq``: The frequency of the animation in Hz
* ``uint8 duration``: The duration of the animation in Seconds.

The ``type`` parameter will map [in order] to one of these animations (`srv/LedAnim.srv <http://docs.ros.org/indigo/api/ardrone_autonomy/html/srv/LedAnim.html>`_ for more details)::

        BLINK_GREEN_RED, BLINK_GREEN, BLINK_RED, BLINK_ORANGE,
        SNAKE_GREEN_RED, FIRE, STANDARD, RED, GREEN, RED_SNAKE,BLANK,
        LEFT_GREEN_RIGHT_RED, LEFT_RED_RIGHT_GREEN, BLINK_STANDARD`

You can test these animations in command line using commands similar to::

    $ rosservice call /ardrone/setledanimation 1 4 5

.. _flightanimation:

Flight Animations
-----------------

.. warning::
    Be extra cautious about using animations, especially flip animations.

Calling ``ardrone/setflightanimation`` service executes one of 20 pre-defined flight animations for the drone. The parameters are:

* ``uint8 type``: The type of flight animation, a number in range [0..19]
* ``uint16 duration``: The duration of the animation. Use 0 for default duration (recommended)

The ``type`` parameter will map [in order] to one of these pre-defined animations (check `srv/FlightAnim.srv <http://docs.ros.org/indigo/api/ardrone_autonomy/html/srv/FlightAnim.html>`_ for more details)::

    ARDRONE_ANIM_PHI_M30_DEG, ARDRONE_ANIM_PHI_30_DEG, ARDRONE_ANIM_THETA_M30_DEG, ARDRONE_ANIM_THETA_30_DEG,
    ARDRONE_ANIM_THETA_20DEG_YAW_200DEG, ARDRONE_ANIM_THETA_20DEG_YAW_M200DEG, ARDRONE_ANIM_TURNAROUND,
    ARDRONE_ANIM_TURNAROUND_GODOWN, ARDRONE_ANIM_YAW_SHAKE, ARDRONE_ANIM_YAW_DANCE, ARDRONE_ANIM_PHI_DANCE,
    ARDRONE_ANIM_THETA_DANCE, ARDRONE_ANIM_VZ_DANCE, ARDRONE_ANIM_WAVE, ARDRONE_ANIM_PHI_THETA_MIXED,
    ARDRONE_ANIM_DOUBLE_PHI_THETA_MIXED, ARDRONE_ANIM_FLIP_AHEAD, ARDRONE_ANIM_FLIP_BEHIND, ARDRONE_ANIM_FLIP_LEFT,
    ARDRONE_ANIM_FLIP_RIGHT

You can test these animations in command line using commands similar to::

    rosservice call /ardrone/setflightanimation 1 0

while drone is flying.

Flat Trim
---------

Calling ``ardrone/flattrim`` service without any parameter will send a "Flat Trim" request to AR-Drone to re-calibrate its rotation estimates assuming that it is on a flat surface. Do not call this service while Drone is flying or while the drone is not actually on a flat surface.

Record to USB Stick
-------------------

Calling ``ardrone/setrecord`` service will enable and disable recording to the USB stick. Pass `1` to enable or `0` to disable this feature.
