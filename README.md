# tamagawa_imu_ros

## Overview

This program is a driver for TAMAGAWA SEIKI IMU.

## Usage

1)Offset cancel reset method

ex)

		rostopic pub -1 /tamagawa_imu/receive_offset_cancel_req std_msgs/Int32 5  

The argument is the number of seconds to average. In this case, offset cancellation is performed on an average of 5 seconds.  

2)Heading angle reset method

ex)

		rostopic pub -1 /tamagawa_imu/receive_heading_reset_req std_msgs/Int32 1  

The arguments can be anything.
