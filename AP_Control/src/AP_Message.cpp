#include "AP_Message.h"

using namespace std;

MSGs::MSGs()
{
    // init
    imuHeading = 0.0;               // IMU Heading
    waypointY = 320.0;                // waypoint
    isParkingAvailable = false;     // 주차할 수 있는지 flag
    isParkingArea = false;          // 주차구역인지 flag
    endParking = false;             // 주차 완료되었는지 flag
    pReverse = false;               // 후진 시작했는지 flag
    endReverse = false;             // 후진 끝나고 다시 차선따라 가는지 flag

    testGear = 2;
    testSpeed = 0;
    testSteer = 0;

    // Publisher
    pub_gear = node_.advertise<std_msgs::Int32> ("/Control/gear", 1);
    pub_speed = node_.advertise<std_msgs::Int32> ("/Control/speed", 1);
    pub_steer = node_.advertise<std_msgs::Float32> ("/Control/steer", 1);

    // Subscriber
    sub_imuHeading = node_.subscribe("/IMU",1, &MSGs::subIMUCallback, this);
    sub_waypointY = node_.subscribe("/Camera/wayPointY",1, &MSGs::subWaypointYCallback, this);
    sub_parking = node_.subscribe("/Camera/parking",1, &MSGs::subParkingCallback, this);
    sub_parkingArea = node_.subscribe("/Camera/traffic/parking",1, &MSGs::subParkingAreaCallback, this);
    //sub_endParking = node_.subscribe("/Camera/endParking",1,&MSGs::subEndParkingCallback, this);
    //sub_pReverse = node_.subscribe("/Camera/pReverse",1, &MSGs::subReverseCallback, this);
    //sub_endReverse = node_.subscribe("/Camera/endReverse",1, &MSGs::subEndReverseCallback, this);

    sub_testGear = node_.subscribe("/test/gear", 1, &MSGs::subTestGearCallback, this);
    sub_testSpeed = node_.subscribe("/test/speed", 1, &MSGs::subTestSpeedCallback, this);
    sub_testSteer = node_.subscribe("/test/steer", 1, &MSGs::subTestSteerCallback, this);
}

// Sub Callback
void MSGs::subIMUCallback(const std_msgs::Float32 &subIMUMsgs)
{
    imuHeading = subIMUMsgs.data;
    //std::cout << "imuHeading : " << imuHeading << std::endl;
}

void MSGs::subWaypointYCallback(const std_msgs::Float32 &subWaypointYMsgs)
{
    waypointY = subWaypointYMsgs.data;
    //std::cout << "waypointY : " << waypointY << std::endl;
}

void MSGs::subParkingCallback(const std_msgs::Bool &subParkingMsgs)
{
    isParkingAvailable = subParkingMsgs.data;
    // std::cout << "isParkingAvailable : " << isParkingAvailable << std::endl;
}

void MSGs::subParkingAreaCallback(const std_msgs::Bool &subParkingAreaMsgs)
{
    //isParkingArea = subParkingAreaMsgs.data;
    isParkingArea = true;   // 임시
    // std::cout << "isParkingArea : " << isParkingArea << std::endl;
}


//void MSGs::subEndParkingCallback(const std_msgs::Bool &subEndParkingMsgs)
//{
//    endParking = subEndParkingMsgs.data;
//     // std::cout << "endParking : " << endParking << std::endl;
//}

//void MSGs::subReverseCallback(const std_msgs::Bool &subReverseMsgs)
//{
//    pReverse = subReverseMsgs.data;
//     // std::cout << "pReverse : " << pReverse << std::endl;
//}

//void MSGs::subEndReverseCallback(const std_msgs::Bool &subEndReverseMsgs)
//{
//    endReverse = subEndReverseMsgs.data;
//    // std::cout << "endReverse : " << endReverse << std::endl;
//}

// TEST ARDUINO
void MSGs::subTestGearCallback(const std_msgs::Int32 &subTestGearMsg)
{
    testGear = subTestGearMsg.data;
}

void MSGs::subTestSpeedCallback(const std_msgs::Int32 &subTestSpeedMsg)
{
    testSpeed = subTestSpeedMsg.data;
}

void MSGs::subTestSteerCallback(const std_msgs::Int32 &subTestSteerMsg)
{
    testSteer = subTestSteerMsg.data;
}
