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

## Usage (Serial)
1)Connect the serial-USB converter to the IMU.


2)Execute command.(Change the launch file according to the IMU model)  

		roslaunch tamagawa_imu_driver serial_AU7554N.launch  
