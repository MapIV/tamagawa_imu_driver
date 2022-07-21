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
 * tag_can_driver.cpp
 * Tamagawa IMU Driver
 * Author MapIV Sekino
 */

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include <chrono>


using namespace std::chrono_literals;

static unsigned int counter;
static int16_t raw_data;
static uint16_t imu_status;
static bool ready = false;

static diagnostic_updater::Updater* p_updater;

static sensor_msgs::msg::Imu imu_msg;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;

void receive_CAN(const can_msgs::msg::Frame::ConstSharedPtr msg){

  if(msg->id == 0x319)
  {
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

    counter = msg->data[1] + (msg->data[0] << 8);
    raw_data = msg->data[3] + (msg->data[2] << 8);
    imu_msg.angular_velocity.x =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    raw_data = msg->data[5] + (msg->data[4] << 8);
    imu_msg.angular_velocity.y =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    raw_data = msg->data[7] + (msg->data[6] << 8);
    imu_msg.angular_velocity.z =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
  }
  if(msg->id == 0x31A)
  {
    imu_status = msg->data[1] + (msg->data[0] << 8);
    raw_data = msg->data[3] + (msg->data[2] << 8);
    imu_msg.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
    raw_data = msg->data[5] + (msg->data[4] << 8);
    imu_msg.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
    raw_data = msg->data[7] + (msg->data[6] << 8);
    imu_msg.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    pub->publish(imu_msg);

    ready = true;
    //std::cout << counter << std::endl;
  }


}

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

  auto now = rclcpp::Clock().now();

  if (now - imu_msg.header.stamp > 1s) {
    level = 2;
    msg = "Message timeout";
  }

  stat.summary(level, msg);
}

void diagnostic_timer_callback()
{
  if(ready)
  {
    p_updater->force_update();
    ready = false;
  }
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("tag_can_driver");

  auto ros_clock = rclcpp::Clock::make_shared();
  auto diagnostics_timer = rclcpp::create_timer(node,ros_clock,1s, &diagnostic_timer_callback);

  diagnostic_updater::Updater updater(node);
  p_updater = &updater;
  updater.setHardwareID("tamagawa");
  updater.add("imu_bit_error", check_bit_error);
  updater.add("imu_connection", check_connection);

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub = node->create_subscription<can_msgs::msg::Frame>("imu/can_tx", 100, receive_CAN);
  pub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 100);
  rclcpp::spin(node);

  return 0;
}
