=========================
Sphero ros package
=========================

A ros packge to control the sphero. This is an adaptation of Melonee Wise's orignial package <https://github.com/mmwise/sphero_ros.git>. To use the spheropy sdk. It is mostly just an remapping to and from sphero commands and messges. A full list of sphero_node topics can be found below. This package a has a gui (sphero_dashboard) for setting sphero colors and heading, and a terminal teleop (sphero_teleop_keyboard).

**Requirements:**

- spheropy


**INSTALL**

.. code-block:: bash

    cd catkin_ws/src

    git clone https://gitlab.com/hcmi/sphero.git

    cd ..

    catkin_make install




**Topics**

*Publishers*

- odom: a nav_msgs/Odometry msg taken from sphero odom values
- imu: a sensor_msgs/Imu msg taken from sphero imu readings
- collision: a SpheroCollision message from the sphero collision detection

*Subscribers*

- cmd_vel: moves the sphero
- cmd_turn: turns the sphero
- set_color: changes the sphero color
- set_back_led: sets the spheros back led
- disable_stabilization: turns of sphero stabilization
- set_heading: rotates the spheros heading
- set_angular_velocity: changes rotation speed
