/*
 * tag_driver.cpp
 * Tamagawa IMU Driver
 * Author Sekino
 * Ver 1.00 2019/3/21
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>

int data_size = 0;
int counter = 0;
int status = 0;
int fd;
int angular_velocity_x_raw = 0;
int angular_velocity_y_raw = 0;
int angular_velocity_z_raw = 0;
int acceleration_x_raw = 0;
int acceleration_y_raw = 0;
int acceleration_z_raw = 0;
double angular_velocity_x = 0.0;
double angular_velocity_y = 0.0;
double angular_velocity_z = 0.0;
double acceleration_x = 0.0;
double acceleration_y = 0.0;
double acceleration_z = 0.0;

int serial_setup(const char *device){
    int fd=open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd, F_SETFL,0);
    struct termios conf_tio;
    tcgetattr(fd,&conf_tio);
    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    conf_tio.c_cc[VMIN]=0;
    conf_tio.c_cc[VTIME]=0;
    tcsetattr(fd,TCSANOW,&conf_tio);
    return fd;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "tag_driver");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    ros::Publisher pub1 = n.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
    std::string imu_type = "withGPS";
    pn.getParam("/tamagawa/imu_type",imu_type);

    //launchにデバイスが定義されているときは、そのデバイスを使用する
    if(argc == 2){
      fd=serial_setup(argv[1]);
        if(fd<0){
            ROS_ERROR("device not found %s",argv[1]);
            ros::shutdown();
        }
    }
    //launchにデバイスが定義されていないときは/dev/ttyUSB0を使用する
    else{
      char device[] ="/dev/ttyUSB0";
      fd=serial_setup(device);
        if(fd<0){
            ROS_ERROR("device not found %s",device);
            ros::shutdown();
        }
    }

    char bin_req[] = "$TSC,BIN,50*04\x0d\x0a";//BINデータを50Hzで出力要求 ※04はチェックサム
    int bin_req_data = write(fd, bin_req, sizeof(bin_req));

    ros::Rate loop_rate(200);
    while (ros::ok()){

      char buf[256];
      int recv_data=read(fd, buf, sizeof(buf));

      if(recv_data>0){
        //BINデータのデコード
        if(buf[0] == '$' &&
           buf[1] == 'T' &&
           buf[2] == 'S' &&
           buf[3] == 'C' &&
           buf[4] == ',' &&
           buf[5] == 'B' &&
           buf[6] == 'I' &&
           buf[7] == 'N' &&
           buf[8] == ','
        ){

            if(strcmp(imu_type.c_str(),"noGPS") == 0){
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

              //ROS_INFO("counter: %d",counter);
              sensor_msgs::Imu imu_msg;
              imu_msg.header.frame_id = "imu";
              imu_msg.header.stamp = ros::Time::now();
              imu_msg.angular_velocity.x = angular_velocity_x;
              imu_msg.angular_velocity.y = angular_velocity_y;
              imu_msg.angular_velocity.z = angular_velocity_z;
              imu_msg.linear_acceleration.x = acceleration_x;
              imu_msg.linear_acceleration.y = acceleration_y;
              imu_msg.linear_acceleration.z = acceleration_z;
              pub1.publish(imu_msg);
            }

            else if(strcmp(imu_type.c_str(),"withGPS") == 0){
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

              //ROS_INFO("counter: %d",counter);
              sensor_msgs::Imu imu_msg;
              imu_msg.header.frame_id = "imu";
              imu_msg.header.stamp = ros::Time::now();
              imu_msg.angular_velocity.x = angular_velocity_x;
              imu_msg.angular_velocity.y = angular_velocity_y;
              imu_msg.angular_velocity.z = angular_velocity_z;
              imu_msg.linear_acceleration.x = acceleration_x;
              imu_msg.linear_acceleration.y = acceleration_y;
              imu_msg.linear_acceleration.z = acceleration_z;
              pub1.publish(imu_msg);
            }
          }
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
