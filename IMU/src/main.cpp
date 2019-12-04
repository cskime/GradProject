#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "AP_IMU.h"

using namespace std;

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "IMU");
    
    ros::NodeHandle node_;
    ros::Publisher publisher = node_.advertise<std_msgs::Float32>("/IMU/heading",1);
    std_msgs::Float32 headingMsg;
    
    IMU imu("/dev/ttyUSB0", 115200);
    
    while(ros::ok())
    {
        headingMsg.data = imu.calculateHeading();
        publisher.publish(headingMsg);
        usleep(3000);
        ros::spinOnce();
    }
    return 0;
}
