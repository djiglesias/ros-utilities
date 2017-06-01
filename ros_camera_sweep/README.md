# ROS Camera Sweeping Assembly #
## Overview ##
This project is intended to extend the functionality of an existing ROS robot by providing a stand alone platform to rotate an Intel RealSense R200 depth camera to increase its field of view. Using a stepper motor and an Arduino Nano, the camera can be rotated through a range of ~130 degrees and the position is published to /tf. For the full project breakdown visit the Instructables.com [HERE](www.poop.com). 

This project assumes that you already have the necessary dependancies installed. This include:
- Realsense_camera
- RTABMAP_ROS
- ROSSERIAL

## Installing to System ##
Navigate to your ROS workspace by typing `roscd` and then into your sandbox. Clone this repository into your workspace, navigate into `ros_camera_sweep`, and build your new ROS package.

~~~
$ git clone git@github.com:djiglesias/ros-utilities.git
$ cd ros-utilities/ros_camera_sweep
$ make
~~~

## Visualizing in RVIZ ##
~~~
$ roslaunch ros_camera_sweep view.launch model:="true" camera:="true"
~~~

## Visualizing in RTABMAP ##
~~~
$ roslaunch ros_camera_sweep rtabmap.launch 
~~~

# Future Work ##
This assembly is functional as a prototype but could be improved in many ways. For instance it could support multiple degrees of free to increase the field of view, contain a protective guard to protect the camera from dirt/dust, and could be more organized with the excessive amount of wiring needed. Feel free to fork this project and improve upon what I have done!
