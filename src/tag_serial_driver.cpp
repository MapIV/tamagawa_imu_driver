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
#include "diagnostic_updater/diagnostic_updater.hpp"
#include <chrono>
#include <cctype>
#include <cstdlib>

using namespace boost::asio;
using namespace std::chrono_literals;

static std::string device = "/dev/ttyUSB0";
static std::string imu_type = "noGPS";
static std::string rate = "50";
static bool ready = false;
static int16_t imu_status;
static sensor_msgs::msg::Imu imu_msg;

static diagnostic_updater::Updater* p_updater;

static void check_bit_error(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  uint8_t level = 0; // OK
  std::string msg = "OK";

  if (imu_status >> 15)
  {
    level = 2; // ERROR
    msg = "Built-In Test error";
  }

  stat.summary(level, msg);
}

static void check_connection(diagnostic_updater::DiagnosticStatusWrapper& stat) 
{
  size_t level = 0; // OK
  std::string msg = "OK";

  auto now = rclcpp::Clock(RCL_ROS_TIME).now();

  if (now - imu_msg.header.stamp > 1s) {
    level = 2;
    msg = "Message timeout";
  }

  stat.summary(level, msg);
}

static void diagnostic_timer_callback()
{
  if(ready)
  {
    p_updater->force_update();
    ready = false;
  }
}

// Frame format (measured on $TSC,RAW output):
//   "$TSC,RAW," | length (2 bytes, big endian) | payload | '*' | checksum (2 hex chars) | CR LF
// length counts the bytes from the length field up to (not including) '*'.
// checksum is the XOR of the bytes between '$' and '*' (NMEA style).
static constexpr size_t kLengthPos = 9;
static constexpr size_t kMinLength = 18;  // the withGPS parser reads up to byte 26
static constexpr size_t kMaxLength = 256;

static bool is_valid_frame(const std::string& buf, size_t star_pos, size_t frame_len)
{
  if (buf[star_pos] != '*' || buf[frame_len - 2] != '\r' || buf[frame_len - 1] != '\n')
  {
    return false;
  }
  if (!isxdigit(buf[star_pos + 1]) || !isxdigit(buf[star_pos + 2]))
  {
    return false;
  }
  uint8_t checksum = 0;
  for (size_t i = 1; i < star_pos; ++i)
  {
    checksum ^= static_cast<uint8_t>(buf[i]);
  }
  return checksum == std::strtol(buf.substr(star_pos + 1, 2).c_str(), nullptr, 16);
}

// Read one frame from the IMU.
// The binary payload may contain 0x0A, so frames are delimited by the header and the length
// field instead of '\n'. Bytes read beyond the returned frame are kept in buf.
static std::string read_frame(serial_port& port, std::string& buf, const std::string& header)
{
  char tmp[256];
  while (true)
  {
    size_t pos = buf.find(header);
    if (pos == std::string::npos)
    {
      // Keep a possibly partial header at the tail
      if (buf.size() >= header.size())
      {
        buf.erase(0, buf.size() - (header.size() - 1));
      }
    }
    else
    {
      buf.erase(0, pos);
      if (buf.size() >= kLengthPos + 2)
      {
        size_t length = (static_cast<uint8_t>(buf[kLengthPos]) << 8) | static_cast<uint8_t>(buf[kLengthPos + 1]);
        size_t star_pos = kLengthPos + length;
        size_t frame_len = star_pos + 5;  // '*' + 2 checksum chars + CR LF
        bool length_ok = (kMinLength <= length && length <= kMaxLength);
        if (!length_ok || buf.size() >= frame_len)
        {
          if (length_ok && is_valid_frame(buf, star_pos, frame_len))
          {
            std::string frame = buf.substr(0, frame_len);
            buf.erase(0, frame_len);
            return frame;
          }
          // Broken frame: drop the leading '$' and resync to the next header
          RCLCPP_WARN(rclcpp::get_logger("tag_serial_driver"), "Invalid frame (length=%zu), resyncing", length);
          buf.erase(0, 1);
          continue;
        }
      }
    }
    size_t n = port.read_some(buffer(tmp));
    buf.append(tmp, n);
  }
}


int main(int argc, char** argv)
{
  auto init_options = rclcpp::InitOptions();
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);
  std::string node_name = "tag_serial_driver";
  auto node = rclcpp::Node::make_shared(node_name);
  auto pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 100);

  auto diagnostics_timer = rclcpp::create_timer(node,node->get_clock(),1s, &diagnostic_timer_callback);
  // auto diagnostics_timer = node->create_wall_timer(1s, &diagnostic_timer_callback);

  node->declare_parameter("use_ros_system",true);
  bool use_ros_system = node->get_parameter("use_ros_system").as_bool();
  RCLCPP_INFO(node->get_logger(), "use_ros_system: %d", use_ros_system);

  diagnostic_updater::Updater updater(node);
  p_updater = &updater;
  updater.setHardwareID("tamagawa");
  updater.add("imu_bit_error", check_bit_error);
  updater.add("imu_connection", check_connection);

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

  imu_msg.header.frame_id = "imu";
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  unsigned int counter;
  int raw_data;

  const std::string header = "$TSC,RAW,";
  std::string serial_buf;

  while (rclcpp::ok())
  {
    //ros::spinOnce();
    std::string rbuf = read_frame(serial_port, serial_buf, header);

    if (rbuf[5] == 'R' && rbuf[6] == 'A' && rbuf[7] == 'W' && rbuf[8] == ',')
    {
      imu_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

      if (strcmp(imu_type.c_str(), "noGPS") == 0)
      {
        counter = ((rbuf[11] << 24) & 0xFF000000) | ((rbuf[12] << 16) & 0x00FF0000) | ((rbuf[13] << 8) & 0x0000FF00) |
                  (rbuf[14] & 0x000000FF);
        imu_status = ((rbuf[13] << 8) & 0xFFFFFF00) | (rbuf[14] & 0x000000FF);
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
        imu_status = ((rbuf[13] << 8) & 0xFFFFFF00) | (rbuf[14] & 0x000000FF);
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

        if (use_ros_system)
        {
          imu_msg.linear_acceleration.y *= -1.0;
          imu_msg.linear_acceleration.z *= -1.0;
          imu_msg.angular_velocity.y *= -1.0;
          imu_msg.angular_velocity.z *= -1.0;
        }

        pub->publish(imu_msg);
      }
      //std::cout << counter << std::endl;
    }
  }
  return 0;
}
