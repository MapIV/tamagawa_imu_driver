// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * tag_serial_driver.cpp
 * Tamagawa IMU Driver
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <boost/asio.hpp>

using namespace boost::asio;

static std::string device = "/dev/ttyUSB0";
static std::string imu_type = "noGPS";
static std::string rate = "50";

int main(int argc, char** argv)
{
  auto init_options = rclcpp::InitOptions();
  init_options.shutdown_on_sigint = false;
  rclcpp::init(argc, argv, init_options);
  std::string node_name = "tag_serial_driver";
  auto node = rclcpp::Node::make_shared(node_name);
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 100);

  io_service io;

  // Use configured device
  if (argc == 4)
  {
    device = argv[1];
    imu_type = argv[2];
    rate = argv[3];
  }
  else if (argc == 3)
  {
    device = argv[1];
    imu_type = argv[2];
  }
  else if (argc == 2)
  {
    device = argv[1];
  }

  std::cout << "device= " << device << " imu_type= " << imu_type << " rate= " << rate << std::endl;

  serial_port serial_port(io, device);
  serial_port.set_option(serial_port_base::baud_rate(115200));
  serial_port.set_option(serial_port_base::character_size(8));
  serial_port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  serial_port.set_option(serial_port_base::parity(serial_port_base::parity::none));
  serial_port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

  // Data output request to IMU
  std::string wbuf = "$TSC,RAW,";
  wbuf += rate;
  wbuf += "\x0d\x0a";
  serial_port.write_some(buffer(wbuf));
  std::cout << "request: " << wbuf << std::endl;

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "imu";
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  unsigned int counter;
  int raw_data;

  while (rclcpp::ok())
  {
    //ros::spinOnce();
    boost::asio::streambuf response;
    boost::asio::read_until(serial_port, response, "\n");
    std::string rbuf(boost::asio::buffers_begin(response.data()), boost::asio::buffers_end(response.data()));

    if (rbuf[5] == 'R' && rbuf[6] == 'A' && rbuf[7] == 'W' && rbuf[8] == ',')
    {
      imu_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

      if (strcmp(imu_type.c_str(), "noGPS") == 0)
      {
        counter = ((rbuf[11] << 24) & 0xFF000000) | ((rbuf[12] << 16) & 0x00FF0000) | ((rbuf[13] << 8) & 0x0000FF00) |
                  (rbuf[14] & 0x000000FF);
        raw_data = ((((rbuf[17] << 8) & 0xFFFFFF00) | (rbuf[18] & 0x000000FF)));
        imu_msg.angular_velocity.x =
            raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[19] << 8) & 0xFFFFFF00) | (rbuf[20] & 0x000000FF)));
        imu_msg.angular_velocity.y =
            raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[21] << 8) & 0xFFFFFF00) | (rbuf[22] & 0x000000FF)));
        imu_msg.angular_velocity.z =
            raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[23] << 8) & 0xFFFFFF00) | (rbuf[24] & 0x000000FF)));
        imu_msg.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[25] << 8) & 0xFFFFFF00) | (rbuf[26] & 0x000000FF)));
        imu_msg.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[27] << 8) & 0xFFFFFF00) | (rbuf[28] & 0x000000FF)));
        imu_msg.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        pub->publish(imu_msg);
      }
      else if (strcmp(imu_type.c_str(), "withGPS") == 0)
      {
        counter = ((rbuf[11] << 8) & 0x0000FF00) | (rbuf[12] & 0x000000FF);
        raw_data = ((((rbuf[15] << 8) & 0xFFFFFF00) | (rbuf[16] & 0x000000FF)));
        imu_msg.angular_velocity.x =
            raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[17] << 8) & 0xFFFFFF00) | (rbuf[18] & 0x000000FF)));
        imu_msg.angular_velocity.y =
            raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[19] << 8) & 0xFFFFFF00) | (rbuf[20] & 0x000000FF)));
        imu_msg.angular_velocity.z =
            raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[21] << 8) & 0xFFFFFF00) | (rbuf[22] & 0x000000FF)));
        imu_msg.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[23] << 8) & 0xFFFFFF00) | (rbuf[24] & 0x000000FF)));
        imu_msg.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[25] << 8) & 0xFFFFFF00) | (rbuf[26] & 0x000000FF)));
        imu_msg.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        pub->publish(imu_msg);
      }
      //std::cout << counter << std::endl;
    }
  }
  return 0;
}
