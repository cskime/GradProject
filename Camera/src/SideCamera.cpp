//
//  SideCamera.cpp
//  GradProjViewer
//
//  Created by cskim on 2019/12/05.
//  Copyright © 2019 cskim. All rights reserved.
//

#include "SideCamera.h"

SideCamera::SideCamera() {
    msgManager = new CameraMessageManager();
    rawImg = msgManager->getSideImage();
    
    // Flags
    isFindSign = false;
    isAvailable = false;
}

/* Find Parking Sign, check isFindSign */

Mat SideCamera::convertImageInRange(Mat image, Scalar lowerBound, Scalar upperBound) {
    Mat HSVImg;
    Rect signROI = Rect(image.cols * 2 / 3,
                        image.rows * 2 / 5,
                        image.cols / 3,
                        image.rows * 3 / 5);
    cvtColor(image(signROI), HSVImg, CV_BGR2HSV);
    
    Mat temp, binary;
    inRange(HSVImg, lowerBound, upperBound, temp);
    dilate(temp, binary, Mat(), Point(-1, -1), 2);
    
    return binary
}

Mat SideCamera::getWhiteSignROI(Mat binaryWhite, int left, int top, int width, int height) {
    Mat whiteROI = binaryWhite;
    for (int y = 0; y < binaryWhite.rows; y++)
    {
        for (int x = 0; x < binaryWhite.cols; x++)
        {
            bool inRange = (y > top && y < (top + height) && x > left && x < (left + width));
            bool isWhite = (binaryWhite.at<uchar>(y,x) == 255);
            
            whiteROI.at<uchar>(y, x) = (inRange && isWhite) ? 255 : 0;
        }
    }
    return whiteROI;
}

int SideCamera::findMaxSignAreaForBlue(Mat binaryBlue, int *left, int *top, int *width, int *height) {
    Mat imgLabels, stats, centroids;
    int numberOfLabels = connectedComponentsWithStats(binaryBlue, imgLabels, stats, centroids, 8, CV_32S);
    
    // Most Big Labeling
    int maxB = -1, idx = 0;
    for (int labelIdx = 1; labelIdx < numberOfLabels; labelIdx++) {
        int area = stats.at<int>(labelIdx, CC_STAT_AREA);
        
        if (maxB < area) {
            maxB = area;
            idx = labelIdx;
        }
    }
    
    float div;
    if (idx != 0) {
        left = stats.at<int>(idx, CC_STAT_LEFT);
        top = stats.at<int>(idx, CC_STAT_TOP);
        width = stats.at<int>(idx, CC_STAT_WIDTH);
        height = stats.at<int>(idx, CC_STAT_HEIGHT);
        
        div = (height != 0) ? (float)(width / height) : 0;
        
        if (div < 0.8 || div > 1.3) {
            left = 0;
            top = 0;
            width = 0;
            height = 0;
        }
    } else {
        left = 0;
        top = 0;
        width = 0;
        height = 0;
    }
    
    return maxB;
}

int SideCamera::findMaxSignAreaForWhite(Mat binaryWhite) {
    Mat imgLabels, stats, centroids;
    int numberOfLabels = connectedComponentsWithStats(binaryWhite, imgLabels, stats, centroids, 8, CV_32S);
    
    int maxW = -1;
    for (int labelIdx = 1; labelIdx < numberOfLabels; labelIdx++) {
        int area = stats.at<int>(labelIdx, CC_STAT_AREA);
        
        if (maxW < area) {
            maxW = area;
            idx = labelIdx;
        }
    }
    
    return maxW;
}

void SideCamera::findParkingSign() {
    // Blue in Sign
    Mat binaryBlue = convertImageInRange(rawImg, Scalar(108, 58, 30), Scalar(123, 216, 150));
    int left, top, width, height;
    int maxB = findMaxSignAreaForBlue(binaryBlue, &left, &top, &width, &height);
    
    // White in Sign
    Mat binaryWhite = convertImageInRange(Scalar(0, 0, 70), Scalar(255, 40, 243));
    Mat whilteROI = getWhiteSignROI(binaryWhite, left, top, width, height);
    int maxW = findMaxSignAreaForWhite(whilteROI);
    
    if (maxW > 1000 && maxB > 2000) {
        isFindSign = true;
        rectangle(rawImg, Point(left, top), Point(left + width, top + height), Scalar(255, 0, 0), 2);
    }
    
    imshow("Sign", rawImg);
    msgManager->publishSign(isFindSign);
}

/* Find Parking Area, check available */

Mat SideCamera::convertToEdge(Mat image) {
    Mat tmpInput = image;
    Mat tmpOutput;
    
    // Origin to Gray
    cvtColor(tmpInput, tmpOutput, CV_BGR2GRAY);
    
    // Gray to Gaussian
    tmpInput = tmpOutput;
    GaussianBlur(tmpInput, tmpOutput, Size(3, 3), 0);
    
    // Gaussian to CannyEdge
    tmpInput = tmpOutput;
    Canny(tmpInput, tmpOutput, 50, 100);
    
    return tmpOutput;
}

bool SideCamera::isInRangeForValue(float theta, float lowerBound, float upperBound) {
    return (theta >= lowerBound) && (theta <= upperBound);
}

Point2d SideCamera::convertPointsToRowTheta(int p1, int p2) {
    // y = ax + b
    double a = (p2.y - p1.y) / (p2.x - p1.x);
    double b = p1.y - a * p1.x;
    double slope = -1 / a;
    
    // cross y = ax + b & y = -1/a x : right_cross
    Point2d cross(-(a*b) / (a*a + 1), b / (a*a + 1));
    Point2d rt(sqrt((cross.x * cross.x) + (cross.y * cross.y)), atan(cross.y / cross.x));
    
    return rt;
}

Point2d SideCamera::average(vector<Point2d> points) {
    double sumX = 0.0, sumY = 0.0;
    
    for(Point2d point : points) {
        sumX += point.x;
        sumY += point.y;
    }
    
    return Point2d(sumX / points.size(), sumY / points.size());
}

vector<Point2d> SideCamera::calculateAverageHoughLines(vector<Vec2f> houghLines, Mat houghImage, bool shouldDraw) {
    // Collection of points
    vector<Point2d> lhsTop, rhsTop;
    vector<Point2d> lhsLeft, rhsLeft;
    vector<Point2d> lhsRight, rhsRight;
    vector<Point2d> lhsBottom, rhsBottom;
    
    // Collect points from detected lines
    for (int lineX = 0; lineX < (int)houghLines.size(); lineX++) {
        // Get rho, theta => x*cos(theta) + y*sin(theta) = rho
        float rho = houghLines[lineX][0];
        float theta = houghLines[lineX][1];
        double cosValue = cos(theta);
        double sinValue = sin(theta);
        
        // Point on houghline
        Point2d center(c * rho, s * rho);
        Point2d delta(1000 * s * -1, 1000 * c);
        Point2d lhs(center - delta);
        Point2d rhs(center + delta);
        
        // Collect each side's point
        if (isInRangeForValue(theta, MIN_THETA_TOP, MAX_THETA_TOP)) {
            lhsTop.push_back(lhs);
            rhsTop.push_back(rhs);
        } else if (isInRangeForValue(theta, MIN_THETA_LEFT, MAX_THETA_LEFT)) {
            lhsLeft.push_back(lhs);
            rhsLeft.push_back(rhs);
        } else if (isInRangeForValue(theta, MIN_THETA_RIGHT, MAX_THETA_RIGHT)) {
            lhsRight.push_back(lhs);
            rhsRight.push_back(rhs);
        } else if (isInRangeForValue(theta, MIN_THETA_BOTTOM, MAX_THETA_BOTTOM)) {
            lhsBottom.push_back(lhs);
            rhsBottom.push_back(rhs);
        } else {
            continue;
        }
    }
    
    // HoughLines with (row, theta) expression
    vector<Point2d> averageHoughLines;
    Point2d rtTop(0, 0), rtLeft(0, 0), rtRight(0, 0), rtBottom(0, 0);
    Point2d averageLhs, averageRhs;
    
    if (lhsTop.size() != 0) {
        averageLhs = average(lhsTop);
        averageRhs = average(rhsTop);
        rtTop = convertPointsToRowTheta(averageLhs, averageRhs);
        if (rtTop.y < 0) {
            rtTop.y += PI;
        }
        if (shouldDraw) {
            cv::line(houghImage, averageLhs, averageRhs, Scalar(255, 0, 0));
        }
    }
    
    if (lhsLeft.size() != 0) {
        averageLhs = average(lhsLeft);
        averageRhs = average(rhsLeft);
        rtLeft = convertPointsToRowTheta(averageLhs, averageRhs);
        
        if (rtLeft.y < 0) {
            rtLeft.y += PI;
        }
        if (shouldDraw) {
            cv::line(houghImage, averageLhs, averageRhs, Scalar(0, 255, 0));
        }
    }
    
    if (lhsRight.size() != 0) {
        averageLhs = average(lhsRight);
        averageRhs = average(rhsRight);
        rtRight = convertPointsToRowTheta(averageLhs, averageRhs);
        
        if (rtRight.y < 0) {
            rtRight.y += PI;
        }
        if (shouldDraw) {
            cv::line(houghImage, averageLhs, averageRhs, Scalar(0, 0, 255));
        }
    }
    
    if (lhsBottom.size() != 0) {
        averageLhs = average(lhsBottom);
        averageRhs = average(rhsBottom);
        rtBottom = convertPointsToRowTheta(averageLhs, averageRhs);
        
        if (rtBottom.y < 0) {
            rtBottom.y += PI;
        }
        if (shouldDraw) {
            cv::line(houghImage, averageLhs, averageRhs, Scalar(0, 255, 255));
        }
    }
    
    // Return collection of houghlines
    averageHoughLines.push_back(rtTop);
    averageHoughLines.push_back(rtLeft);
    averageHoughLines.push_back(rtRight);
    averageHoughLines.push_back(rtBottom);
                                
    return averageHoughLines;
}

Point2d SideCamera::calculateIntersectionFromLines(Point2d line1, Point2d line2) {
    // Denominator, numerator
    double denX, numX, interX;
    numX = line2.x / sin(line2.y) - line1.x / sin(line1.y);
    denX = -cos(line1.y) / sin(line1.y) + cos(line2.y) / sin(line2.y);
    interX = numX / denX;
    
    double denY, numY, interY;
    numY = line1.x - interX * cos(line1.y);
    denY = sin(line1.y);
    interY = numY / denY;
    
    return Point2d(interX, interY);
}

vector<Point2d> SideCamera::getVerticesFromHoughLines(vector<Point2d> houghLines, Mat houghImage, bool shouldDraw) {
    vector<Point2d> intersectionSet;
    if (isInRangeForValue(houghLines.size(), 1, 4)) {
        // vector of houghlines sequence : top, left, right, bottom
        for (int idx1 = 0; idx1 < houghLines.size() - 1; idx1++) {
            for (int idx2 = idx1 + 1; idx2 < houghLines.size(); idx2++) {
                Point2d intersection = calculateIntersectionFromLines(houghLines[idx1], houghLines[idx2]);
                if (isInRangeForValue(intersection.x, 0, houghImage.cols) &&
                    isInRangeForValue(intersection.y, 30, houghImage.rows)) {
                    intersectionSet.push_back(intersection);
                    
                    if (shouldDraw) {
                        cv::circle(houghImage, intersection, 2, Scalar(0, 0, 255), 2);
                    }
                }
            }
        }
    }
    return intersectionSet;
}

Mat SideCamera::getWarpedImageFromImage(Mat edgeImage, Size warpSize, vector<Point2d> intersections) {
    Mat warpImage(warpSize, edgeImage.type());
    
    if (intersections.size() == 4)
    {
        vector<Point2f> corners(4);
        for (int i = 0; i < 4; i++)
        {
            corners[i] = intersections[i];
        }

        //Warping Point
        vector<Point2f> warpCorners(4);
        warpCorners[0] = Point2f(0, 0);
        warpCorners[1] = Point2f(warpImage.cols, 0);
        warpCorners[2] = Point2f(0, warpImage.rows);
        warpCorners[3] = Point2f(warpImage.cols, warpImage.rows);

        //Transformation Matrix
        Mat trans = getPerspectiveTransform(corners, warpCorners);

        //Warping
        warpPerspective(edgeImage, warpImage, trans, warpSize);
        
    } else {
        warpImg.setTo(Scalar(0));
    }
    
    return warpImage;
}

bool SideCamera::canParkedWithinArea(Mat edgeImage, int intersections) {
    
    Mat warpImage = getWarpedImageFromImage(edgeImage, Size(200, 200), intersections);
        
    int edge_count = 0;
    row = warpImage.rows;
    col = warpImage.cols;

    for (int r = row * 0.5; r <= row * 0.8; r++) {
        for (int c = col * 0.2; c <= col * 0.8; c++) {
            if (warpImage.at<uchar>(r, c) == 255) {
                edge_count++;
            }
        }
    }

    return (edge_count > 0);
}

void SideCamera::findParkingArea() {
    // Get Edge Image
    Rect ROI = Rect(0, 0, rawImg.cols * 2 / 3, rawImg.rows);
    Mat imageROI = rawImage(ROI);
    Mat edgeImg = convertToEdge(imageROI);
    
    // Get Hough Lines
    vector<Vec2f> houghLines;
    HoughLines(edgeImg, houghLines, 1, PI / 180, 100);
    
    // Houghlines 평균
    Mat houghImg;
    cvtColor(edgeImg, houghImg, CV_GRAY2BGR);
    vector<Point2d> averageHoughLines = calculateAverageHoughLines(houghImg, houghLines, false);
    
    // 네 개의 꼭지점 얻기
    vector<Point2d> intersections = getVerticesFromHoughLines(averageHoughLines);
    
    // edge counting using warp image
    bool canParked = canParkedWithinArea(edgeImg, intersections);
    
    msgManager->publishParkingAvailable(canParked);
}
