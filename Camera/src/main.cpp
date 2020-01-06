#include <iostream>

#include "CameraController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Camera");

    // Instance
    CameraManager manager;

    // ROS
    while(ros::ok())
    {
        manager.process();
        ros::spinOnce();
    }
    return 0;
}
