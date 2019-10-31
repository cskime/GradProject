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
    sub_waypointY = node_.subscribe("/Camera/wayPointY",1, &ParkingMessagePool::subWaypointYCallback, this);
    sub_isAvailableFront = node_.subscribe("/Camera/parkingFront",1, &ParkingMessagePool::subParkingFrontCallback, this);
    sub_isAvailableRear = node_.subscribe("/Camera/traffic/parking",1, &ParkingMessagePool::subParkingRearCallback, this);
    sub_isComplete = node_.subscribe("/Camera/endParking",1,&ParkingMessagePool::subEndParkingCallback, this);
    sub_isReturned = node_.subscribe("/Camera/endReverse",1, &ParkingMessagePool::subEndReverseCallback, this);
}

/* Subscriber Callback */
void ControllerMessageManager::subIMUCallback(const std_msgs::Float32 &subIMUMsgs)
{
    heading = subIMUMsgs;
}

void ControllerMessageManager::subWaypointYCallback(const std_msgs::Float32 &subWaypointYMsgs)
{
    waypointY = subWaypointYMsgs;
}

void ControllerMessageManager::subParkingFrontCallback(const std_msgs::Bool &subParkingFrontMsgs)
{
    isAvailableFront = subParkingFrontMsgs;
}

void ControllerMessageManager::subParkingRearCallback(const std_msgs::Bool &subParkingRearMsgs)
{
    isAvailableRear = subParkingRearMsgs;
}

void ControllerMessageManager::subCompleteCallback(const std_msgs::Bool &subCompleteCallbackMsgs)
{
    isComplete = subCompleteCallbackMsgs;
}

void ControllerMessageManager::subReturnedCallback(const std_msgs::Bool &subReturnedMsgs)
{
    isReturned = subReturnedMsgs;
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

bool ControllerMessageManager::isValidFront() {
    return isAvailableFront.data;
}

bool ControllerMessageManager::isValidRear() {
    return isAvailableRear.data;
}

bool ControllerMessageManager::isComplete() {
    return isCompleteMsg.data;
}

bool ControllerMessageManager::isReturned() {
    return isReturnedMsg.data;
}
