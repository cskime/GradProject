//
//  SideCamera.cpp
//  GradProjViewer
//
//  Created by cskim on 2019/12/05.
//  Copyright © 2019 cskim. All rights reserved.
//

#include "FrontCamera.h"

FrontCamera::FrontCamera() {
    // Flags
    wayPoint = Point2f(0.0f, 0.0f);
    msgManager = new CameraMessageManager();
    rawImage = msgManager->getFrontImage();
}

Mat FrontCamera::reverseBinary(Mat binaryImage) {
    // binary_reverse
    Mat reversed = Mat::zeros(binaryImage.rows, binaryImage.cols ,CV_8UC1);
    for (int row = 0; row < binaryImage.rows; row++)
    {
        for (int col = 0; col < binaryImage.cols; col++)
        {
            reversed.at<uchar>(row, col) = 255 - binaryImage.at<uchar>(row, col);
        }
    }
    return reversed;
}

void FrontCamera::calculateWayPoint() {
    Mat grayImage, binaryImage;
    cvtColor(rawImage, grayImage, CV_BGR2GRAY);
    threshold(grayImage, binaryImage, 0, 255, CV_THRESH_OTSU);

    // Reversed Binary Image
    Mat reversedBinary = reverseBinary(binaryImage);

    // mophology
//    erode(reversedBinary, reversedBinary, getStructuringElement((MORPH_ELLIPSE), Size(3, 3)));
    dilate(reversedBinary, reversedBinary, getStructuringElement((MORPH_ELLIPSE), Size(3, 3)));

    int Line_L_X = 0;
    int Line_R_X = m_binary.cols - 1;
    float line_width = 0;

    for (int y = 390; y > 320; y--)
    {
        /////////////////////////Left Lane////////////////////////
        for (int x = (reversedBinary.cols / 2) - 1; x >= 0; x--)
        {
            uchar P = reversedBinary.at<uchar>(y, x);

            if (P == 255)
            {
                if (Line_L_X < x)
                {
                    Line_L_X = x;
                }
            }
        }

        /////////////////////////Right Lane////////////////////////
        for (int x = (reversedBinary.cols / 2) - 1; x < m_roi.br().x; x++)
        {
            uchar P = reversedBinary.at<uchar>(y, x);

            if (P == 255)
            {
                if (Line_R_X > x)
                {
                    Line_R_X = x;
                }
            }
        }

        if(Line_L_X != 0 && Line_R_X != (reversedBinary.cols-1) && (Line_R_X-Line_L_X) > 150)
        {
            wayPoint = Point((Line_L_X + Line_R_X)/2, y);
            line_width = Line_R_X-Line_L_X;
            circle(reversedBinary, wayPoint, 5, Scalar(255), 5);
            break;
        }
    }

    msgManager->publishWay(wayPoint.x);
}
