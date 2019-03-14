
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int data_size = 0;
int counter = 0;
int status = 0;
double angular_velocity_x = 0.0;


int open_serial(const char *device_name){
    int fd1=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd1, F_SETFL,0);
    //load configuration
    struct termios conf_tio;
    tcgetattr(fd1,&conf_tio);
    //set baudrate
    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    //non canonical, non echo back
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    //non blocking
    conf_tio.c_cc[VMIN]=0;
    conf_tio.c_cc[VTIME]=0;
    //store configuration
    tcsetattr(fd1,TCSANOW,&conf_tio);
    return fd1;
}

int fd1;
void serial_callback(const std_msgs::String& serial_msg){
    int rec=write(fd1,serial_msg.data.c_str(),serial_msg.data.size());
    if(rec>=0)printf("send:%s\n",serial_msg.data.c_str());
    else{
        ROS_ERROR_ONCE("Serial Fail: cound not write");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_driver");
    ros::NodeHandle n;

    //Publisher
    ros::Publisher serial_pub = n.advertise<std_msgs::String>("Serial_in", 1000);

    //Subscriber
    ros::Subscriber serial_sub = n.subscribe("Serial_out", 10, serial_callback);

    char device_name[]="/dev/ttyUSB0";
    fd1=open_serial(device_name);

    if(fd1<0){
        ROS_ERROR("Serial Fail: cound not open %s", device_name);
        printf("Serial Fail\n");
        ros::shutdown();
    }

    ros::Rate loop_rate(200);
    while (ros::ok()){

      char buf[256];
      int recv_data=read(fd1, buf, sizeof(buf));

      if(recv_data>0){
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

        data_size = ((buf[9] << 8) & 0x0000FF00) + (buf[10] & 0x000000FF);
        //ROS_INFO("data_size: %d [byte]",data_size);

        counter = ((buf[11] << 8) & 0x0000FF00) + (buf[12] & 0x000000FF);
        ROS_INFO("counter: %d",counter);

        status = ((buf[13] << 8) & 0x0000FF00) + (buf[14] & 0x000000FF);
        //ROS_INFO("status: %d",status);

        //angular_velocity_x = (((buf[15] << 8) & 0x0000FF00) + (buf[16] & 0x000000FF)) * (200/pow(2,15));
        int test = (((buf[15] << 8) & 0x0000FF00) + (buf[16] & 0x000000FF)) * (200/pow(2,15));
        ROS_INFO("%x",test);
        ROS_INFO("%x %x",buf[15],buf[16]);
        }

            std_msgs::String serial_msg;
            serial_msg.data=buf;
            serial_pub.publish(serial_msg);

        }

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
