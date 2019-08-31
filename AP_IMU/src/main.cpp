#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>


#include <fstream>
#include <queue>

#include "AP_IMU.h"
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "IMU");

    IMU IM("/dev/ttyUSB0", 115200);

    ros::NodeHandle node_;
    ros::Publisher pub_imu = node_.advertise<std_msgs::Float32>("/IMU",1);
    std_msgs::Float32 angle;

    while(ros::ok())
     {
        IM.IMUProcessing();
        angle.data = IM.angle_z;
        pub_imu.publish(angle);
        cout << IM.angle_z << endl;

        usleep(3000);
        ros::spinOnce();
    }
    return 0;
}
