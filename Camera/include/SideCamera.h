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

class SideCamera {
    Mat rawImg;
    
    /* Find Parking Sign */
    
    bool isFindSign;
    
    Mat convertImageInRange(Scalar lowerBound, Scalar upperBound);
    Mat getWhiteSignROI(Mat binaryWhite, int left, int top, int width, int height);
    int findMaxSignAreaForBlue(Mat binaryBlue, int *left, int *top, int *width, int *height);  // idx를 pointer로 넘겨서 외부 변수에 값을 넣으려고 함
    int findMaxSignAreaForWhite(Mat binaryWhite);
    
    
    /* Find Parking Area */
    
    bool isAvailable;
    float MIN_THETA_TOP = 1.21, MAX_THETA_TOP = 1.25;
    float MIN_THETA_LEFT = 2.20, MAX_THETA_LEFT = 2.24;
    float MIN_THETA_RIGHT = 1.79, MAX_THETA_RIGHT = 1.81;
    float MIN_THETA_BOTTOM = 0.20, MAX_THETA_BOTTOM = 0.40;
    
    Mat convertImageToEdge();
    bool isInRangeForValue(float theta, float lowerBound, float upperBound);
    Point2d convertPointsToRowTheta(Point2d p1, Point2d p2);
    Point2d average(vector<Point2d> points);
    vector<Point2d> calculateAverageHoughLines(vector<Vec2f> houghLines, Mat houghImage, bool shouldDraw);
    Point2d calculateIntersectionFromLines(Point2d line1, Point2d line2);
    vector<Point2d> getVerticesFromHoughLines(vector<Point2d> houghLines, Mat houghImage, bool shouldDraw);
    bool canParkedWithinArea(Mat edgeImage, vector<Point2d> intersections);
    Mat getWarpedImageFromImage(Mat edgeImage, Size warpSize, vector<Point2d> intersections);
    
public:
    SideCamera(Mat rawImg);
    ~SideCamera() {
        delete msgManager;
    }
    
    bool findParkingSign();     // Find Parking Sign
    bool findParkingArea();     // Find Parking Area
};
