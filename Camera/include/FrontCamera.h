#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <geometry_msgs/Point.h>

#include <iostream>
#include <vector>

#include "CameraMessageManager.h"
#include "CameraCommon.h"

class FrontCamera {
    Mat rawImage;
    
    Point2f wayPoint;
    Mat reverseBinary(Mat binaryImage);
    
public:
    FrontCamera(Mat rawImg);
    ~FrontCamera() {
        delete msgManager;
    }
    
    float calculateWayPoint();
};
