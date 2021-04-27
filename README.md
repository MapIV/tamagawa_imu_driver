# tamagawa_imu_driver (Beta version)

[![CircleCI](https://circleci.com/gh/MapIV/tamagawa_imu_driver/tree/master.svg?style=svg)](https://circleci.com/gh/MapIV/tamagawa_imu_driver/tree/master)

## Overview

This package contains ROS driver for TAMAGAWA SEIKI IMUs.

## Install

This package requires can_msgs.

		sudo apt-get install ros-foxy-can-msgs  

## Usage (CAN)

1)Use [ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan) etc. to publish the IMU CAN signal with the topic name /can/imu with the data type can_msgs/Frame.msg.  

2)Execute command.  

		ros2 launch tamagawa_imu_driver can.launch  

## Usage (Serial)
1)Connect the serial-USB converter to the IMU.


2)Execute command.(Change the launch file according to the IMU model)  

		ros2 launch tamagawa_imu_driver serial_AU7554N.launch  
