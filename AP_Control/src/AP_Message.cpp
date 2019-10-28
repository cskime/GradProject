#include "AP_Message.h"

using namespace std;

ParkingMessagePool::ParkingMessagePool()
{
    /* initialize */
    // Publisher
    pub_gear = node_.advertise<std_msgs::Int32> ("/Control/gear", 1);
    pub_velocity = node_.advertise<std_msgs::Int32> ("/Control/velocity", 1);
    pub_steer = node_.advertise<std_msgs::Float32> ("/Control/steer", 1);

    // Subscriber : message 이름 수정필요, subscriber 추가/삭제 검토 필요
    sub_heading = node_.subscribe("/IMU",1, &ParkingMessagePool::subIMUCallback, this);
    sub_waypointY = node_.subscribe("/Camera/wayPointY",1, &ParkingMessagePool::subWaypointYCallback, this);
    sub_isAvailableFront = node_.subscribe("/Camera/parkingFront",1, &ParkingMessagePool::subParkingFrontCallback, this);
    sub_isAvailableRear = node_.subscribe("/Camera/traffic/parking",1, &ParkingMessagePool::subParkingRearCallback, this);
    sub_isComplete = node_.subscribe("/Camera/endParking",1,&ParkingMessagePool::subEndParkingCallback, this);
    sub_isReturnRail = node_.subscribe("/Camera/pReverse",1, &ParkingMessagePool::subReverseCallback, this);
    sub_isReturned = node_.subscribe("/Camera/endReverse",1, &ParkingMessagePool::subEndReverseCallback, this);
}

/* Subscriber Callback */
void ParkingMessagePool::subIMUCallback(const std_msgs::Float32 &subIMUMsgs)
{
    heading = subIMUMsgs;
}

void ParkingMessagePool::subWaypointYCallback(const std_msgs::Float32 &subWaypointYMsgs)
{
    waypointY = subWaypointYMsgs;
}

void ParkingMessagePool::subParkingFrontCallback(const std_msgs::Bool &subParkingFrontMsgs)
{
    isAvailableFront = subParkingFrontMsgs;
}

void ParkingMessagePool::subParkingRearCallback(const std_msgs::Bool &subParkingRearMsgs)
{
    isAvailableRear = subParkingRearMsgs;
}

void ParkingMessagePool::subCompleteCallback(const std_msgs::Bool &subCompleteCallbackMsgs)
{
    isComplete = subCompleteCallbackMsgs;
}

void ParkingMessagePool::subReturnRailCallback(const std_msgs::Bool &subReturnRailMsgs)
{
    isReturnRail = subReturnRailMsgs;
}

void ParkingMessagePool::subReturnedCallback(const std_msgs::Bool &subReturnedMsgs)
{
    isReturned = subReturnedMsgs;
}

/* Publisher */
void ParkingMessagePool::publish(int gear, int velocity, float steer) {
    this.gear.data = gear;
    this.velocity.data = velocity;
    this.steer.data = steer;
    
    pub_gear.publish(this.gear);
    pub_velocity.publish(this.velocity);
    pub_steer.publish(this.steer);
}
