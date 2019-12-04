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


#define PI 3.141592


using namespace cv;
using namespace std;
using namespace ros;

// Parking
class CCamera
{
private:
    Mat m_front, m_side;
    Mat m_gray;
    Mat m_gauss;
    Mat m_binary;
    Mat m_canny;
    Mat m_hough;

    Rect m_roi;

    float min_left, max_left;
    float min_right, max_right;
    float min_up, max_up;
    float min_bottom, max_bottom;

    vector<Vec2f> lines;
    vector<Point2d> hough_const;
    vector<Point2d> cp;

public:
    CCamera();
    void Processing();
    void frontProcessing();
    void sideProcessing();

    void findSign();
    void findParkingArea();

    Point2f m_wayPoint;     // wayX
    bool m_parkingSign;     // is parking Sign
    bool m_parkingArea;     // is parking Area
    bool m_endParking;      // is end parking
    bool m_parkingReverse;  // is reverse driven
    bool m_endReverse;      // is end reverse

    // ROS property
    ros::NodeHandle nh;

    /* Multiple Camera
     * camera1/usb_cam1 : front view
     * camera2/usb_cam2 : parking view
     */
    // Image Pub/Sub
    ros::Subscriber subFrontImage;
    ros::Subscriber subParkingImage;
    ros::Publisher pub_front;
    ros::Publisher pub_side;

    // Flag publishing
    ros::Publisher pub_waypointY;       // wayX
    ros::Publisher pub_parkingSign;     // is parking Sign
    ros::Publisher pub_parkingArea;     // is parking Area
    ros::Publisher pub_endParking;      // is end parking
    ros::Publisher pub_pReverse;        // is reverse driven
    ros::Publisher pub_endReverse;      // is end reverse

    // MSGS
    std_msgs::Float32 wayX;             // wayX
    std_msgs::Bool isParkingSign;       // is parking Sign
    std_msgs::Bool isParkingArea;       // is parking Area
    std_msgs::Bool isEndParking;        // is end parking
    std_msgs::Bool isReverse;           // is reverse driven
    std_msgs::Bool isEndReverse;        // is end reverse

    // ROS imagePtr
    cv_bridge::CvImagePtr rawImagePtr, rawImagePtr1, rawImagePtr2;
    cv_bridge::CvImagePtr pubImagePtr, pubImagePtr1, pubImagePtr2;
    cv::Mat rawImage, rawImagefront, rawImageside;

    // Callback
    void subImgCallback(const sensor_msgs::Image& subImgMsgs);
    void subImgCallback1(const sensor_msgs::Image& subImgMsgs);
    void subImgCallback2(const sensor_msgs::Image& subImgMsgs);
};

Point2d twoPoint2rhotheta(Point2d p1, Point2d p2);
Point2d calc_crosspoint(Point2d rt1, Point2d rt2);
void averageLine(vector<Point2d> p1, vector<Point2d> p2, Point2d& avgp1, Point2d& avgp2, int count);
