# tamagawa_imu_driver (Beta version)

[![CircleCI](https://circleci.com/gh/MapIV/tamagawa_imu_driver/tree/master.svg?style=svg)](https://circleci.com/gh/MapIV/tamagawa_imu_driver/tree/master)

## Overview

This package contains ROS driver for TAMAGAWA SEIKI IMUs.

## Usage

1)Offset cancel reset method

ex)

		rostopic pub -1 /tamagawa_imu/receive_offset_cancel_req std_msgs/Int32 5  

The argument is the number of seconds to average. In this case, offset cancellation is performed on an average of 5 seconds.  

2)Heading angle reset method

ex)

		rostopic pub -1 /tamagawa_imu/receive_heading_reset_req std_msgs/Int32 1  

The arguments can be anything.
