#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <queue>

#include "AP_Camera.h"

#include <std_msgs/Float32.h>

using namespace cv;
using namespace std;
using namespace ros;

typedef         unsigned char           u8;
typedef         unsigned short          u16;
typedef         unsigned int            u32;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Camera");

    // Instance
    CCamera apCam;

    // ROS
    while(ros::ok())
    {
        if(!apCam.rawImagefront.empty())
        {
            apCam.Processing();

            waitKey(10);
        }
        ros::spinOnce();
    }
    return 0;
}
