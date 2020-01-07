#include "ControllerMessageManager.h"

using namespace std;

ControllerMessageManager::ControllerMessageManager()
{
    /* initialize */
    // Publisher
    pub_gear = node_.advertise<std_msgs::Int32> ("/Control/gear", 1);
    pub_velocity = node_.advertise<std_msgs::Int32> ("/Control/velocity", 1);
    pub_steer = node_.advertise<std_msgs::Float32> ("/Control/steer", 1);

    // Subscriber : message 이름 수정필요, subscriber 추가/삭제 검토 필요
    sub_heading = node_.subscribe("/IMU/heading",1, &ParkingMessagePool::subIMUCallback, this);
    sub_waypointX = node_.subscribe("/Camera/wayPointX",1, &ParkingMessagePool::subWaypointXCallback, this);
    sub_isFindSign = node_.subscribe("/Camera/isFindSign",1, &ParkingMessagePool::subFindSignCallback, this);
    sub_isAvailable = node_.subscribe("/Camera/isAvailable",1, &ParkingMessagePool::subAvailableCallback, this);
}

/* Subscriber Callback */

void ControllerMessageManager::subIMUCallback(const std_msgs::Float32 &subIMUMsgs) {
    heading = subIMUMsgs;
}

void ControllerMessageManager::subWaypointXCallback(const std_msgs::Float32 &subWaypointXMsgs) {
    waypointX = subWaypointXMsgs;
}

void ControllerMessageManager::subFindSignCallback(const std_msgs::Bool &subFindSignMsgs) {
    isFindSign = subFindSignMsgs;
}

void ControllerMessageManager::subAvailableCallback(const std_msgs::Bool &subAvailableMsgs) {
    isAvailable = subAvailableMsgs;
}

/* Public  */
void ControllerMessageManager::publish(int gear, int velocity, float steer) {
    std_msgs::Int32 gear;               // Arduino로 보낼 기어값
    std_msgs::Int32 velocity;           // Arduino로 보낼 속도값
    std_msgs::Float32 steer;            // Arduino로 보낼 조향각
    
    gear.data = gear;
    velocity.data = velocity;
    steer.data = steer;
    
    pub_gear.publish(gear);
    pub_velocity.publish(velocity);
    pub_steer.publish(steer);
}

float ControllerMessageManager::getWayPointX() {
    return wayPointX;
}

float ControllerMessageManager::getHeading() {
    return heading;
}

bool ControllerMessageManager::isFindParkingSign() {
    return isFindSign;
}

bool ControllerMessageManager::isParkingAvailable() {
    return isAvailable;
}
