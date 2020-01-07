#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>
#include <ctime>

#include "CameraMessageManager.h"
#include "SideCamera.h"
#include "FrontCamera.h"
#include "CameraCommon.h"

using namespace cv;
using namespace std;
using namespace ros;

class CameraManager {
private:
    CameraMessageManager *msgManager;
    SideCamera *sideCamera;
    FrontCamera *frontCamera;
    
public:
    CameraManager();
    ~CameraManager() {
        delete sideCamera;
        delete frontCamera;
    }
    
    void process();
};
