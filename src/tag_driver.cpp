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
 * tag_driver.cpp
 * Tamagawa IMU Driver
 * Author MapIV Sekino
 * Ver 1.00 2019/4/4
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "std_msgs/Int32.h"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <signal.h>

// Parameter default value
std::string frame_id = "imu";
std::string imu_type = "noGPS";
std::string rate = "100";
bool publish_air_pressure = "false";
std::string imu_topic_name = "/imu/data_raw";
std::string air_pressure_topic_name = "/air_pressure";

struct termios old_conf_tio;
struct termios conf_tio;

int fd;
int data_size = 0;
int counter = 0;
int status = 0;
int angular_velocity_x_raw = 0;
int angular_velocity_y_raw = 0;
int angular_velocity_z_raw = 0;
double angular_velocity_x = 0.0;
double angular_velocity_y = 0.0;
double angular_velocity_z = 0.0;
int acceleration_x_raw = 0;
int acceleration_y_raw = 0;
int acceleration_z_raw = 0;
double acceleration_x = 0.0;
double acceleration_y = 0.0;
double acceleration_z = 0.0;
int roll_angle_raw = 0;
int pitch_angle_raw = 0;
int heading_angle_raw = 0;
double roll_angle = 0.0;
double pitch_angle = 0.0;
double heading_angle = 0.0;
int fluid_pressure_raw = 0;
double fluid_pressure = 0.0;
int pressure_altitude_raw = 0;
double pressure_altitude = 0.0;

sensor_msgs::Imu imu_msg;
sensor_msgs::FluidPressure pressure_msg;

int serial_setup(const char* device)
{
  int fd = open(device, O_RDWR | O_NOCTTY);
  fcntl(fd, F_SETFL, 0);
  tcgetattr(fd, &conf_tio);

  speed_t BAUDRATE = B115200;
  cfsetispeed(&conf_tio, BAUDRATE);
  cfsetospeed(&conf_tio, BAUDRATE);

  conf_tio.c_cflag |= CREAD | CLOCAL;
  conf_tio.c_iflag &= ~IGNBRK;
  conf_tio.c_lflag = 0;
  conf_tio.c_oflag = 0;
  conf_tio.c_cc[VMIN] = 1;
  conf_tio.c_cc[VTIME] = 0;
  conf_tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  conf_tio.c_cflag |= (CLOCAL | CREAD);
  conf_tio.c_cflag &= ~(PARENB | PARODD);
  conf_tio.c_cflag |= 0;
  conf_tio.c_cflag &= ~CSTOPB;
  conf_tio.c_cflag &= ~CRTSCTS;
  conf_tio.c_iflag &= ~(ICRNL | IGNCR | INLCR);
  conf_tio.c_oflag &= ~(ONLCR | OCRNL);

  tcsetattr(fd, TCSANOW, &conf_tio);

  // Settings used before
  /*
  int fd=open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  fcntl(fd, F_SETFL,0);
  tcgetattr(fd,&conf_tio);
  speed_t BAUDRATE = B115200;
  cfsetispeed(&conf_tio, BAUDRATE);
  cfsetospeed(&conf_tio, BAUDRATE);
  conf_tio.c_lflag &= ~(ECHO | ICANON);
  conf_tio.c_cc[VMIN]=1;  //Do not use inter-character timer
  conf_tio.c_cc[VTIME]=0;
  tcsetattr(fd,TCSANOW,&conf_tio);
  */

  return fd;
}

void receive_ver_req(const std_msgs::Int32::ConstPtr& msg)
{
  char ver_req[] = "$TSC,VER*29\x0d\x0a";
  int ver_req_data = write(fd, ver_req, sizeof(ver_req));
  ROS_INFO("Send Version Request:%s", ver_req);
}

void receive_offset_cancel_req(const std_msgs::Int32::ConstPtr& msg)
{
  char offset_cancel_req[32];
  sprintf(offset_cancel_req, "$TSC,OFC,%d\x0d\x0a", msg->data);
  int offset_cancel_req_data = write(fd, offset_cancel_req, sizeof(offset_cancel_req));
  ROS_INFO("Send Offset Cancel Request:%s", offset_cancel_req);
}

void receive_heading_reset_req(const std_msgs::Int32::ConstPtr& msg)
{
  char heading_reset_req[] = "$TSC,HRST*29\x0d\x0a";
  int heading_reset_req_data = write(fd, heading_reset_req, sizeof(heading_reset_req));
  ROS_INFO("Send Heading reset Request:%s", heading_reset_req);
}

void shutdown_cmd(int sig)
{
  tcsetattr(fd, TCSANOW, &old_conf_tio);  // Revert to previous settings
  close(fd);
  ROS_INFO("Port closed");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tag_driver", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.getParam("/tamagawa_imu/frame_id", frame_id);
  pn.getParam("/tamagawa_imu/type", imu_type);
  pn.getParam("/tamagawa_imu/rate", rate);
  pn.getParam("/tamagawa_imu/publish_air_pressure", publish_air_pressure);
  pn.getParam("/tamagawa_imu/imu_topic_name", imu_topic_name);
  pn.getParam("/tamagawa_imu/air_pressure_topic_name", air_pressure_topic_name);

  ros::Publisher pub1 = n.advertise<sensor_msgs::Imu>(imu_topic_name, 1000);
  ros::Publisher pub2 = n.advertise<sensor_msgs::FluidPressure>(air_pressure_topic_name, 1000);
  ros::Subscriber sub1 = n.subscribe("/tamagawa_imu/receive_ver_req", 10, receive_ver_req);
  ros::Subscriber sub2 = n.subscribe("/tamagawa_imu/receive_offset_cancel_req", 10, receive_offset_cancel_req);
  ros::Subscriber sub3 = n.subscribe("/tamagawa_imu/receive_heading_reset_req", 10, receive_heading_reset_req);

  tcgetattr(fd, &old_conf_tio);  // Last setting value retention

  // Use configured device
  if (argc == 2)
  {
    fd = serial_setup(argv[1]);
    if (fd < 0)
    {
      ROS_ERROR("device not found %s", argv[1]);
      ros::shutdown();
    }
  }
  // Device when not set
  else
  {
    char device[] = "/dev/ttyUSB0";
    fd = serial_setup(device);
    if (fd < 0)
    {
      ROS_ERROR("device not found %s", device);
      ros::shutdown();
    }
  }

  // Data output request to IMU
  // Model without barometric pressure sensor uses BIN
  if (publish_air_pressure == false)
  {
    char lvl_req[] = "$TSC,LVL*3E\x0d\x0a";  // Command operation in leveling mode
    int lvl_req_data = write(fd, lvl_req, sizeof(lvl_req));
    ros::Duration(0.5).sleep();
    char bin_req[32];
    sprintf(bin_req, "$TSC,BIN,%s\x0d\x0a", rate.c_str());
    int bin_req_data = write(fd, bin_req, sizeof(bin_req));
    ROS_INFO("request:%s", bin_req);
  }

  // Model with barometric pressure sensor uses BIN2
  else if (publish_air_pressure == true)
  {
    char lvl_req[] = "$TSC,LVL*3E\x0d\x0a";  // Command operation in leveling mode
    int lvl_req_data = write(fd, lvl_req, sizeof(lvl_req));
    ros::Duration(0.5).sleep();
    char bin_req[32];
    sprintf(bin_req, "$TSC,BIN2,%s\x0d\x0a", rate.c_str());
    int bin_req_data = write(fd, bin_req, sizeof(bin_req));
    ROS_INFO("request:%s", bin_req);
  }

  ros::Duration(0.5).sleep();
  ros::Rate loop_rate(500);
  while (ros::ok())
  {
    char buf[255];
    int recv_data = read(fd, buf, sizeof(buf));

    // ROS_INFO("%s",buf);

    if (recv_data > 0)
    {
      if (strcmp(imu_type.c_str(), "noGPS") == 0)
      {
        if (buf[5] == 'B' && buf[6] == 'I' && buf[7] == 'N' && buf[8] == ',')
        {
          data_size = ((buf[9] << 8) & 0x0000FF00) | (buf[10] & 0x000000FF);
          counter = ((buf[11] << 24) & 0xFF000000) | ((buf[12] << 16) & 0x00FF0000) | ((buf[13] << 8) & 0x0000FF00) |
                    (buf[14] & 0x000000FF);
          status = ((buf[15] << 8) & 0x0000FF00) | (buf[16] & 0x000000FF);
          angular_velocity_x_raw = ((((buf[17] << 8) & 0xFFFFFF00) | (buf[18] & 0x000000FF)));
          angular_velocity_x =
              angular_velocity_x_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
          angular_velocity_y_raw = ((((buf[19] << 8) & 0xFFFFFF00) | (buf[20] & 0x000000FF)));
          angular_velocity_y =
              angular_velocity_y_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
          angular_velocity_z_raw = ((((buf[21] << 8) & 0xFFFFFF00) | (buf[22] & 0x000000FF)));
          angular_velocity_z =
              angular_velocity_z_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
          acceleration_x_raw = ((((buf[23] << 8) & 0xFFFFFF00) | (buf[24] & 0x000000FF)));
          acceleration_x = acceleration_x_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
          acceleration_y_raw = ((((buf[25] << 8) & 0xFFFFFF00) | (buf[26] & 0x000000FF)));
          acceleration_y = acceleration_y_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
          acceleration_z_raw = ((((buf[27] << 8) & 0xFFFFFF00) | (buf[28] & 0x000000FF)));
          acceleration_z = acceleration_z_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
          roll_angle_raw = ((((buf[29] << 8) & 0xFFFFFF00) | (buf[30] & 0x000000FF)));
          roll_angle = roll_angle_raw * (180 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg] => [rad]
          pitch_angle_raw = ((((buf[31] << 8) & 0xFFFFFF00) | (buf[32] & 0x000000FF)));
          pitch_angle = pitch_angle_raw * (180 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg] => [rad]
          heading_angle_raw = ((((buf[33] << 8) & 0xFFFFFF00) | (buf[34] & 0x000000FF)));
          heading_angle = heading_angle_raw * (180 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg] => [rad]

          // sensor_msgs::Imu imu_msg;
          imu_msg.header.frame_id = frame_id;
          imu_msg.header.stamp = ros::Time::now();
          imu_msg.orientation.x = 0.0;
          imu_msg.orientation.y = 0.0;
          imu_msg.orientation.z = 0.0;
          imu_msg.orientation.w = 1.0;
          imu_msg.angular_velocity.x = angular_velocity_x;
          imu_msg.angular_velocity.y = angular_velocity_y;
          imu_msg.angular_velocity.z = angular_velocity_z;
          imu_msg.linear_acceleration.x = acceleration_x;
          imu_msg.linear_acceleration.y = acceleration_y;
          imu_msg.linear_acceleration.z = acceleration_z;
          pub1.publish(imu_msg);
          ROS_INFO("counter: %d", counter);
        }
      }
      else if (strcmp(imu_type.c_str(), "withGPS") == 0 && publish_air_pressure == false)
      {
        if (buf[5] == 'B' && buf[6] == 'I' && buf[7] == 'N' && buf[8] == ',')
        {
          int counter2 = counter;

          data_size = ((buf[9] << 8) & 0x0000FF00) | (buf[10] & 0x000000FF);
          counter = ((buf[11] << 8) & 0x0000FF00) | (buf[12] & 0x000000FF);
          status = ((buf[13] << 8) & 0x0000FF00) | (buf[14] & 0x000000FF);
          angular_velocity_x_raw = ((((buf[15] << 8) & 0xFFFFFF00) | (buf[16] & 0x000000FF)));
          angular_velocity_x =
              angular_velocity_x_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
          angular_velocity_y_raw = ((((buf[17] << 8) & 0xFFFFFF00) | (buf[18] & 0x000000FF)));
          angular_velocity_y =
              angular_velocity_y_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
          angular_velocity_z_raw = ((((buf[19] << 8) & 0xFFFFFF00) | (buf[20] & 0x000000FF)));
          angular_velocity_z =
              angular_velocity_z_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
          acceleration_x_raw = ((((buf[21] << 8) & 0xFFFFFF00) | (buf[22] & 0x000000FF)));
          acceleration_x = acceleration_x_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
          acceleration_y_raw = ((((buf[23] << 8) & 0xFFFFFF00) | (buf[24] & 0x000000FF)));
          acceleration_y = acceleration_y_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
          acceleration_z_raw = ((((buf[25] << 8) & 0xFFFFFF00) | (buf[26] & 0x000000FF)));
          acceleration_z = acceleration_z_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
          roll_angle_raw = ((((buf[27] << 8) & 0xFFFFFF00) | (buf[28] & 0x000000FF)));
          roll_angle = roll_angle_raw * (180 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg] => [rad]
          pitch_angle_raw = ((((buf[29] << 8) & 0xFFFFFF00) | (buf[30] & 0x000000FF)));
          pitch_angle = pitch_angle_raw * (180 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg] => [rad]
          heading_angle_raw = ((((buf[31] << 8) & 0xFFFFFF00) | (buf[32] & 0x000000FF)));
          heading_angle = heading_angle_raw * (180 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg] => [rad]

          // sensor_msgs::Imu imu_msg;
          imu_msg.header.frame_id = frame_id;
          imu_msg.header.stamp = ros::Time::now();
          imu_msg.orientation.x = 0.0;
          imu_msg.orientation.y = 0.0;
          imu_msg.orientation.z = 0.0;
          imu_msg.orientation.w = 1.0;
          imu_msg.angular_velocity.x = angular_velocity_x;
          imu_msg.angular_velocity.y = angular_velocity_y;
          imu_msg.angular_velocity.z = angular_velocity_z;
          imu_msg.linear_acceleration.x = acceleration_x;
          imu_msg.linear_acceleration.y = acceleration_y;
          imu_msg.linear_acceleration.z = acceleration_z;
          pub1.publish(imu_msg);
          ROS_INFO("counter: %d", counter - counter2);
        }
      }
      else if (strcmp(imu_type.c_str(), "withGPS") == 0 && publish_air_pressure == true)
      {
        if (buf[5] == 'B' && buf[6] == 'I' && buf[7] == 'N' && buf[8] == '2' && buf[9] == ',')
        {
          data_size = ((buf[10] << 8) & 0x0000FF00) | (buf[11] & 0x000000FF);
          counter = ((buf[12] << 8) & 0x0000FF00) | (buf[13] & 0x000000FF);
          status = ((buf[14] << 8) & 0x0000FF00) | (buf[15] & 0x000000FF);
          angular_velocity_x_raw = ((((buf[16] << 8) & 0xFFFFFF00) | (buf[17] & 0x000000FF)));
          angular_velocity_x =
              angular_velocity_x_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
          angular_velocity_y_raw = ((((buf[18] << 8) & 0xFFFFFF00) | (buf[19] & 0x000000FF)));
          angular_velocity_y =
              angular_velocity_y_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
          angular_velocity_z_raw = ((((buf[20] << 8) & 0xFFFFFF00) | (buf[21] & 0x000000FF)));
          angular_velocity_z =
              angular_velocity_z_raw * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
          acceleration_x_raw = ((((buf[22] << 8) & 0xFFFFFF00) | (buf[23] & 0x000000FF)));
          acceleration_x = acceleration_x_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
          acceleration_y_raw = ((((buf[24] << 8) & 0xFFFFFF00) | (buf[25] & 0x000000FF)));
          acceleration_y = acceleration_y_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
          acceleration_z_raw = ((((buf[26] << 8) & 0xFFFFFF00) | (buf[27] & 0x000000FF)));
          acceleration_z = acceleration_z_raw * (100 / pow(2, 15));  // LSB & unit [m/s^2]
          roll_angle_raw = ((((buf[28] << 8) & 0xFFFFFF00) | (buf[29] & 0x000000FF)));
          roll_angle = roll_angle_raw * (180 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg] => [rad]
          pitch_angle_raw = ((((buf[30] << 8) & 0xFFFFFF00) | (buf[31] & 0x000000FF)));
          pitch_angle = pitch_angle_raw * (180 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg] => [rad]
          heading_angle_raw = ((((buf[32] << 8) & 0xFFFFFF00) | (buf[33] & 0x000000FF)));
          heading_angle = heading_angle_raw * (180 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg] => [rad]

          fluid_pressure_raw = ((((buf[54] << 8) & 0xFFFFFF00) | (buf[55] & 0x000000FF)));
          fluid_pressure = fluid_pressure_raw * (1500 / pow(2, 15));  // LSB & unit [hPa]
          pressure_altitude_raw = ((((buf[56] << 8) & 0xFFFFFF00) | (buf[57] & 0x000000FF)));
          pressure_altitude = pressure_altitude_raw * (5000 / pow(2, 15));  // LSB & unit [m]
          // ROS_INFO("pressure_altitude: %lf",pressure_altitude);

          imu_msg.header.frame_id = frame_id;
          imu_msg.header.stamp = ros::Time::now();
          imu_msg.orientation.x = 0.0;
          imu_msg.orientation.y = 0.0;
          imu_msg.orientation.z = 0.0;
          imu_msg.orientation.w = 1.0;
          imu_msg.angular_velocity.x = angular_velocity_x;
          imu_msg.angular_velocity.y = angular_velocity_y;
          imu_msg.angular_velocity.z = angular_velocity_z;
          imu_msg.linear_acceleration.x = acceleration_x;
          imu_msg.linear_acceleration.y = acceleration_y;
          imu_msg.linear_acceleration.z = acceleration_z;
          pub1.publish(imu_msg);

          pressure_msg.header.frame_id = frame_id;
          pressure_msg.header.stamp = ros::Time::now();
          pressure_msg.fluid_pressure = fluid_pressure;
          pressure_msg.variance = 0;
          pub2.publish(pressure_msg);
          ROS_INFO("counter: %d", counter);
        }
      }
      if (buf[5] == 'V' && buf[6] == 'E' && buf[7] == 'R' && buf[8] == ',')
      {
        ROS_INFO("%s", buf);
      }
    }

    signal(SIGINT, shutdown_cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
