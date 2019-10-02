# tamagawa_imu_driver (Beta version)

[![CircleCI](https://circleci.com/gh/MapIV/tamagawa_imu_driver/tree/master.svg?style=svg)](https://circleci.com/gh/MapIV/tamagawa_imu_driver/tree/master)

## Overview

This package contains ROS driver for TAMAGAWA SEIKI IMUs.

## Install

This package requires can_msgs.

		sudo apt-get install ros-kinetic-can-msgs  

## Usage (CAN)

1)Use [socketcan_interface](http://wiki.ros.org/socketcan_interface) etc. to publish the IMU CAN signal with the topic name /can/imu with the data type can_msgs/Frame.msg.  

2)Execute command.  

		roslaunch tamagawa_imu_driver can.launch  

## Usage (Serial) Not recommended!
1)Connect the serial-USB converter to the IMU.


2)Execute command.(Change the launch file according to the IMU model)  

		roslaunch tamagawa_imu_driver serial_AU7554N.launch  

3)The output frequency can be changed with rosparam.  

		/tamagawa_imu/frame_id　= frame_id  #Set the required frame id  
		/tamagawa_imu/type = "noGPS" or "withGPS" #Set according to IMU model  
		/tamagawa_imu/rate = 0~200 #Output frequency varies depending on IMU model  
		/tamagawa_imu/publish_air_pressure = true or false #Whether it can be output depends on the IMU model  

4)Offset cancel reset method.

ex)

		rostopic pub -1 /tamagawa_imu/receive_offset_cancel_req std_msgs/Int32 5  

The argument is the number of seconds to average. In this case, offset cancellation is performed on an average of 5 seconds.  

5)Heading angle reset method.

ex)

		rostopic pub -1 /tamagawa_imu/receive_heading_reset_req std_msgs/Int32 1  

The arguments can be anything.
