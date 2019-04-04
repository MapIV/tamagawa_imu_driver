/*
 * tag_driver.cpp
 * Tamagawa IMU Driver
 * Author MapIV Sekino
 * Ver 1.00 2019/4/4
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>

//Parameter default value
std::string frame_id = "imu";
std::string imu_type = "noGPS";
std::string rate = "100";
bool publish_air_pressure = "false";
std::string imu_topic_name = "/imu/data_raw";
std::string air_pressure_topic_name = "/air_pressure";

int fd;
//int data_size = 0;
int counter = 0;
//int status = 0;
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

int fluid_pressure_raw = 0;
double fluid_pressure = 0.0;

int serial_setup(const char *device){
    int fd=open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd, F_SETFL,0);
    struct termios conf_tio;
    tcgetattr(fd,&conf_tio);
    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    conf_tio.c_cc[VMIN]=1;  //Do not use inter-character timer
    conf_tio.c_cc[VTIME]=0;
    tcsetattr(fd,TCSANOW,&conf_tio);
    return fd;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "tag_driver");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pn.getParam("/tamagawa_imu/frame_id",frame_id);
    pn.getParam("/tamagawa_imu/type",imu_type);
    pn.getParam("/tamagawa_imu/rate",rate);
    pn.getParam("/tamagawa_imu/publish_air_pressure",publish_air_pressure);
    pn.getParam("/tamagawa_imu/imu_topic_name",imu_topic_name);
    pn.getParam("/tamagawa_imu/air_pressure_topic_name",air_pressure_topic_name);

    ros::Publisher pub1 = n.advertise<sensor_msgs::Imu>(imu_topic_name, 1000);
    ros::Publisher pub2 = n.advertise<sensor_msgs::Imu>(air_pressure_topic_name, 1000);

    //Use configured device
    if(argc == 2){
      fd=serial_setup(argv[1]);
        if(fd<0){
            ROS_ERROR("device not found %s",argv[1]);
            ros::shutdown();
        }
    }
    //Device when not set
    else{
      char device[] ="/dev/ttyUSB0";
      fd=serial_setup(device);
        if(fd<0){
            ROS_ERROR("device not found %s",device);
            ros::shutdown();
        }
    }

    //Data output request to IMU

    //Model without barometric pressure sensor uses BIN
    if(publish_air_pressure == false){
      char bin_req[32];
      sprintf(bin_req, "$TSC,BIN,%s\x0d\x0a",rate.c_str());
      int bin_req_data = write(fd, bin_req, sizeof(bin_req));
      ROS_INFO("request:%s",bin_req);
    }
    //Model with barometric pressure sensor uses BIN2
    else if(publish_air_pressure == true){
      char bin_req[32];
      sprintf(bin_req, "$TSC,BIN2,%s\x0d\x0a",rate.c_str());
      int bin_req_data = write(fd, bin_req, sizeof(bin_req));
      ROS_INFO("request:%s",bin_req);
    }

    ros::Rate loop_rate(500);
    while (ros::ok()){

      char buf[255];
      int recv_data=read(fd, buf, sizeof(buf));

      //ROS_INFO("%s",buf);

      if(recv_data > 0){

        if(strcmp(imu_type.c_str(),"noGPS") == 0){
            if(buf[5] == 'B' &&
               buf[6] == 'I' &&
               buf[7] == 'N' &&
               buf[8] == ','
            ){
            //data_size = ((buf[9] << 8) & 0x0000FF00) | (buf[10] & 0x000000FF);
            counter = ((buf[11] << 24) & 0xFF000000) | ((buf[12] << 16) & 0x00FF0000) | ((buf[13] << 8) & 0x0000FF00) | (buf[14] & 0x000000FF);
            //status = ((buf[15] << 8) & 0x0000FF00) | (buf[16] & 0x000000FF);
            angular_velocity_x_raw = ((((buf[17] << 8) & 0xFFFFFF00) | (buf[18] & 0x000000FF)));
            angular_velocity_x = angular_velocity_x_raw * (200/pow(2,15)) * M_PI / 180; //LSB & unit [deg/s] => [rad/s]
            angular_velocity_y_raw = ((((buf[19] << 8) & 0xFFFFFF00) | (buf[20] & 0x000000FF)));
            angular_velocity_y = angular_velocity_y_raw * (200/pow(2,15)) * M_PI / 180; //LSB & unit [deg/s] => [rad/s]
            angular_velocity_z_raw = ((((buf[21] << 8) & 0xFFFFFF00) | (buf[22] & 0x000000FF)));
            angular_velocity_z = angular_velocity_z_raw * (200/pow(2,15)) * M_PI / 180; //LSB & unit [deg/s] => [rad/s]
            acceleration_x_raw = ((((buf[23] << 8) & 0xFFFFFF00) | (buf[24] & 0x000000FF)));
            acceleration_x = acceleration_x_raw * (100/pow(2,15)); //LSB & unit [m/s^2]
            acceleration_y_raw = ((((buf[25] << 8) & 0xFFFFFF00) | (buf[26] & 0x000000FF)));
            acceleration_y = acceleration_y_raw * (100/pow(2,15)); //LSB & unit [m/s^2]
            acceleration_z_raw = ((((buf[27] << 8) & 0xFFFFFF00) | (buf[28] & 0x000000FF)));
            acceleration_z = acceleration_z_raw * (100/pow(2,15)); //LSB & unit [m/s^2]

            sensor_msgs::Imu imu_msg;
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
            //ROS_INFO("counter: %d",counter);
          }
        }
        else if(strcmp(imu_type.c_str(),"withGPS") == 0 && publish_air_pressure == false){
            if(buf[5] == 'B' &&
               buf[6] == 'I' &&
               buf[7] == 'N' &&
               buf[8] == ','
            ){
            //data_size = ((buf[9] << 8) & 0x0000FF00) | (buf[10] & 0x000000FF);
            counter = ((buf[11] << 8) & 0x0000FF00) | (buf[12] & 0x000000FF);
            //status = ((buf[13] << 8) & 0x0000FF00) | (buf[14] & 0x000000FF);
            angular_velocity_x_raw = ((((buf[15] << 8) & 0xFFFFFF00) | (buf[16] & 0x000000FF)));
            angular_velocity_x = angular_velocity_x_raw * (200/pow(2,15)) * M_PI / 180; //LSB & unit [deg/s] => [rad/s]
            angular_velocity_y_raw = ((((buf[17] << 8) & 0xFFFFFF00) | (buf[18] & 0x000000FF)));
            angular_velocity_y = angular_velocity_y_raw * (200/pow(2,15)) * M_PI / 180; //LSB & unit [deg/s] => [rad/s]
            angular_velocity_z_raw = ((((buf[19] << 8) & 0xFFFFFF00) | (buf[20] & 0x000000FF)));
            angular_velocity_z = angular_velocity_z_raw * (200/pow(2,15)) * M_PI / 180; //LSB & unit [deg/s] => [rad/s]
            acceleration_x_raw = ((((buf[21] << 8) & 0xFFFFFF00) | (buf[22] & 0x000000FF)));
            acceleration_x = acceleration_x_raw * (100/pow(2,15)); //LSB & unit [m/s^2]
            acceleration_y_raw = ((((buf[23] << 8) & 0xFFFFFF00) | (buf[24] & 0x000000FF)));
            acceleration_y = acceleration_y_raw * (100/pow(2,15)); //LSB & unit [m/s^2]
            acceleration_z_raw = ((((buf[25] << 8) & 0xFFFFFF00) | (buf[26] & 0x000000FF)));
            acceleration_z = acceleration_z_raw * (100/pow(2,15)); //LSB & unit [m/s^2]

            sensor_msgs::Imu imu_msg;
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
            //ROS_INFO("counter: %d",counter);
          }
        }
        else if(strcmp(imu_type.c_str(),"withGPS") == 0 && publish_air_pressure == true){
            if(buf[5] == 'B' &&
               buf[6] == 'I' &&
               buf[7] == 'N' &&
               buf[8] == '2' &&
               buf[9] == ','
            ){
            //data_size = ((buf[9] << 8) & 0x0000FF00) | (buf[10] & 0x000000FF);
            counter = ((buf[11] << 8) & 0x0000FF00) | (buf[12] & 0x000000FF);
            //status = ((buf[13] << 8) & 0x0000FF00) | (buf[14] & 0x000000FF);
            angular_velocity_x_raw = ((((buf[15] << 8) & 0xFFFFFF00) | (buf[16] & 0x000000FF)));
            angular_velocity_x = angular_velocity_x_raw * (200/pow(2,15)) * M_PI / 180; //LSB & unit [deg/s] => [rad/s]
            angular_velocity_y_raw = ((((buf[17] << 8) & 0xFFFFFF00) | (buf[18] & 0x000000FF)));
            angular_velocity_y = angular_velocity_y_raw * (200/pow(2,15)) * M_PI / 180; //LSB & unit [deg/s] => [rad/s]
            angular_velocity_z_raw = ((((buf[19] << 8) & 0xFFFFFF00) | (buf[20] & 0x000000FF)));
            angular_velocity_z = -1 * angular_velocity_z_raw * (200/pow(2,15)) * M_PI / 180; //LSB & unit [deg/s] => [rad/s]
            acceleration_x_raw = ((((buf[21] << 8) & 0xFFFFFF00) | (buf[22] & 0x000000FF)));
            acceleration_x = -1 * acceleration_x_raw * (100/pow(2,15)); //LSB & unit [m/s^2]
            acceleration_y_raw = ((((buf[23] << 8) & 0xFFFFFF00) | (buf[24] & 0x000000FF)));
            acceleration_y = -1 * acceleration_y_raw * (100/pow(2,15)); //LSB & unit [m/s^2]
            acceleration_z_raw = ((((buf[25] << 8) & 0xFFFFFF00) | (buf[26] & 0x000000FF)));
            acceleration_z = acceleration_z_raw * (100/pow(2,15)); //LSB & unit [m/s^2]
            fluid_pressure_raw = ((((buf[55] << 8) & 0x0000FF00) | (buf[56] & 0x000000FF)));
            fluid_pressure = fluid_pressure_raw * (1500/pow(2,15)); //LSB & unit [hPa]

            sensor_msgs::Imu imu_msg;
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

            sensor_msgs::FluidPressure pressure_msg;
            pressure_msg.header.frame_id = frame_id;
            pressure_msg.header.stamp = ros::Time::now();
            pressure_msg.fluid_pressure = fluid_pressure;
            pressure_msg.variance = 0;
            pub2.publish(pressure_msg);
            //ROS_INFO("counter: %d",counter);
          }
        }
      }

      ros::spinOnce();
      loop_rate.sleep();

    }
    return 0;
}
