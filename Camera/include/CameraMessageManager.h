#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <iostream>

using namespace cv;
using namespace std;
using namespace ros;


class CameraMessageManager {
private:
    
    // ROS property
    ros::NodeHandle nh;

    /* Multiple Camera
     * camera1/usb_cam1 : front view
     * camera2/usb_cam2 : parking view
     */
   
    /* Subscriber */
    ros::Subscriber subFrontImage;
    ros::Subscriber subSideImage;
    
    cv_bridge::CvImagePtr rawImagePtr, rawImagePtr1, rawImagePtr2;
    cv_bridge::CvImagePtr pubImagePtr, pubImagePtr1, pubImagePtr2;
    cv::Mat rawImagefront, rawImageside;
    
    void subFrontImageCallback(const sensor_msgs::Image& subImgMsgs);
    void subSideImageCallback(const sensor_msgs::Image& subImgMsgs);
    
    /* Publisher */
    
    ros::Publisher pub_waypointX;           // wayX
    ros::Publisher pub_isFindSign;          // is parking Sign
    ros::Publisher pub_isAvailable;         // is parking available
    
    // MSGS
    std_msgs::Float32 wayX;             // wayX
    std_msgs::Bool isFindSign;       // is parking Sign
    std_msgs::Bool isAvailable;       // is parking Area
    
public:
    CameraMessageManager();
    void publishWay(float waypointX);
    void publishSign(float findSign);
    void publishParkingAvailable(float available);
    Mat getFrontImage();
    Mat getSideImage();
};
