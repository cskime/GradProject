
#include "CameraMessageManager.h"

CameraMessageManager::CameraMessageManager() {
    subFrontImage = nh.subscribe("/camera1/usb_cam1/image_raw", 1, &CameraMessageManager::subFrontImageCallback, this);
    subSideImage = nh.subscribe("/camera2/usb_cam2/image_raw", 1, &CameraMessageManager::subSideImageCallback, this);
    
    pub_waypointY = nh.advertise<std_msgs::Float32>("/Camera/wayPointY", 1);
    pub_isFindSign = nh.advertise<std_msgs::Bool>("/Camera/isFindSign", 1);
    pub_isAvailable = nh.advertise<std_msgs::Bool>("/Camera/isAvailable", 1);'
}

void CameraMessageManager::subFrontImageCallback(const sensor_msgs::Image& subImgMsgs)
{
    if(subImgMsgs.data.size())
    {
        rawImagePtr1 = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImagefront = rawImagePtr1->image;

        rawImagePtr1->image = rawImagefront;
        //pubImage.publish(rawImagePtr->toImageMsg());
    }
}

void CameraMessageManager::subSideImageCallback(const sensor_msgs::Image& subImgMsgs)
{
    if(subImgMsgs.data.size())
    {
        rawImagePtr2 = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImageside = rawImagePtr2->image;

        rawImagePtr2->image = rawImageside;
        //pubImage.publish(rawImagePtr->toImageMsg());
    }
}

void CameraMessageManager::publishWay(float waypointX) {
    wayX.data = wayPointX;
    pub_waypointX.publish(wayX);
}

void CameraMessageManager::publishSign(bool findSign) {
    isFindSign.data = isFindSign;
    pub_isFindSign.publish(isFindSign);
}

void CameraMessageManager::publishAvail(bool available) {
    isAvailable.data = available;
    pub_isAvailable.publish(isAvailable);
}

