#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

class MSGs {
public:

    MSGs();

    /* Variables */
    float imuHeading;
    float waypointY;
    bool isParkingAvailable;
    bool isParkingArea;
    bool endParking;
    bool pReverse;
    bool endReverse;

    int testGear, testSpeed, testSteer;

    ros::NodeHandle node_;

    /* Publisher */
    ros::Publisher pub_gear;    // 전진(D), 후진(R), 정지(P)
    ros::Publisher pub_speed;   // speed = 150
    ros::Publisher pub_steer;   // steering calculated with wayX

    /* Subscriber */
    ros::Subscriber sub_imuHeading;     // IMU Heading
    ros::Subscriber sub_waypointY;      // wayX
    ros::Subscriber sub_parking;        // isParkingAvailable(주차공간)
    ros::Subscriber sub_parkingArea;    // isParkingArea(표지판)
    //ros::Subscriber sub_endParking;     // 주차완료
    //ros::Subscriber sub_pReverse;       // 후진
    //ros::Subscriber sub_endReverse;     // 후진 완료 후 전진

    ros::Subscriber sub_testGear;
    ros::Subscriber sub_testSpeed;
    ros::Subscriber sub_testSteer;

    /* Publish msgs */
    std_msgs::Int32 gear;
    std_msgs::Int32 speed;
    std_msgs::Float32 steer;

    /* Subscriber Callback */
    void subIMUCallback(const std_msgs::Float32 &subIMUMsgs);
    void subWaypointYCallback(const std_msgs::Float32 &subWaypointYMsgs);
    void subParkingCallback(const std_msgs::Bool &subParkingMsgs);
    void subParkingAreaCallback(const std_msgs::Bool &subParkingAreaMsgs);
    //void subEndParkingCallback(const std_msgs::Bool &subEndParkingMsgs);
    //void subReverseCallback(const std_msgs::Bool &subReverseMsgs);
    //void subEndReverseCallback(const std_msgs::Bool &subEndReverseMsgs);

    void subTestGearCallback(const std_msgs::Int32 &subTestGearMsg);
    void subTestSpeedCallback(const std_msgs::Int32 &subTestSpeedMsg);
    void subTestSteerCallback(const std_msgs::Int32 &subTestSteerMsg);

};
