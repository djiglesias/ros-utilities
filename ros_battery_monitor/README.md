# ROS Battery Monitor #
## Overview ##
This project is intended to extend the functionality of an existing ROS robot by providing a stand alone platform for monitoring the health of a lithium polymer (lipo) battery with up to 12 cells. The voltage of each individual cell is read and published to a ROS topic as a sensor_msgs/BatteryState message at 1Hz. Additionally, the unit indicated whether or not a battery is plugged in and reports on the health of the battery. For the full project breakdown visit the Instructables.com [HERE](https://www.instructables.com/id/1S-6S-Battery-Voltage-Monitor-ROS/). 

This project assumes that you already have the necessary dependancies installed for using ROSSERIAL.

## Installing to System ##
Navigate to your ROS workspace by typing `roscd` and then into your sandbox. Clone this repository into your workspace, navigate into `ros_battery_monitor`, and build your new ROS package.

~~~
$ git clone git@github.com:djiglesias/ros-utilities.git
$ cd ros-utilities/ros_battery_monitor
$ make
~~~

## Visualizing ROS Topic ##
Start the battery monitor and view the topic output.
~~~
$ roslaunch ros_battery_monitor battery_monitor.launch &
$ rostopic echo /battery/info
~~~

# Future Work ##
This assembly is functional as a prototype but could be improved in many ways. For instance it could:
- Support current draw monitoring to warn the user of current overdraw errors.
- Utilize a 16 channel multiplexer for reading inputs of individual cells on a single pin.
- Could support battery charging when present.