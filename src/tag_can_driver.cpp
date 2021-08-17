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

#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "sensor_msgs/Imu.h"
#include "diagnostic_updater/diagnostic_updater.h"

static unsigned int counter;
static int16_t raw_data;
static int32_t raw_data_fog;
static uint16_t imu_status;
static bool use_fog;

static diagnostic_updater::Updater* p_updater;

static sensor_msgs::Imu imu_msg;
static ros::Publisher pub;

void receive_can_callback(const can_msgs::Frame::ConstPtr& msg){

  if(msg->id == 0x319)
  {
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = msg->header.stamp;

    counter = msg->data[1] + (msg->data[0] << 8);
    raw_data = msg->data[3] + (msg->data[2] << 8);
    imu_msg.angular_velocity.x =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    raw_data = msg->data[5] + (msg->data[4] << 8);
    imu_msg.angular_velocity.y =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    
    if (use_fog) 
    {
      raw_data_fog = (msg->data[7] + (msg->data[6] << 8)) + ((msg->data[5] << 16) + (msg->data[4] << 24));
      imu_msg.angular_velocity.z =
        raw_data_fog * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    }
    else 
    {
      raw_data = msg->data[7] + (msg->data[6] << 8);
      imu_msg.angular_velocity.z =
        raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
    }
    
  }
  if(msg->id == 0x31A)
  {
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
    pub.publish(imu_msg);
    //std::cout << counter << std::endl;
  }

  p_updater->update();

}

void diagnostic_tag_can(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  ros::Time diagnostic_time = ros::Time::now();

  if(diagnostic_time.toSec() - imu_msg.header.stamp.toSec() >= 5.0)
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::STALE, "No data coming in for more than 1s");
  }
  else if(imu_status & 0x8000)
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Error detected by BIT");
  }
  else
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv, "tag_can_driver");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  pnh.getParam("tag_can_driver/use_fog", use_fog);

  diagnostic_updater::Updater updater;
  p_updater = &updater;
  updater.setHardwareID("tamagawa_imu");
  updater.add("tamagawa_imu", diagnostic_tag_can);
  
  ros::Subscriber sub = n.subscribe("imu/can_tx", 100, receive_can_callback);
  pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
  ros::spin();

  return 0;
}
