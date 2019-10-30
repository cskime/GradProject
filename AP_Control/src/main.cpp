#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <fstream>
#include <queue>

#include "ControlManager.h"

using namespace cv;
using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control");

    ControlManager manager;
    
    // init
    int drivingSpeed = 140;
    int parkingSpeed = 120;

    while(ros::ok())
    {
        manager.parking();
        usleep(31000);
        ros::spinOnce();
    }
}
