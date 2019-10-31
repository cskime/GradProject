#include <ros/ros.h>
#include <iostream>

#include "Controller.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control");

    Controller controller;

    while(ros::ok())
    {
        controller.parking();
        usleep(31000);
        ros::spinOnce();
    }
}
