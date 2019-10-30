#include <iostream>
#include <ros/ros.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>

class ControlMessageManager {
    
    /* common */
    ros::NodeHandle node_;
    
    /* publish */
    ros::Publisher pub_gear;        // 전진(D), 후진(R), 정지(P)
    ros::Publisher pub_velocity;    // speed = 150
    ros::Publisher pub_steer;       // steering calculated with wayX
    
    /* subscribe */
    ros::Subscriber sub_heading;
    ros::Subscriber sub_waypointY;
    ros::Subscriber sub_isAvailableFront;
    ros::Subscriber sub_isAvailableRear;
    ros::Subscriber sub_isComplete;
    ros::Subscriber sub_isReturnRail;
    ros::Subscriber sub_isReturned;
    
public:
    // Messages
    std_msgs::Int32 gear;               // Arduino로 보낼 기어값
    std_msgs::Int32 velocity;           // Arduino로 보낼 속도값
    std_msgs::Float32 steer;            // Arduino로 보낼 조향각
    
    std_msgs::Float32 heading;          // IMU heading
    std_msgs::Float32 waypointY;        // waypoint
    std_msgs::Bool isAvailableFront;    // 전면주차공간 찾음 & 주차 가능
    std_msgs::Bool isAvailableRear;     // 후면주차공간 찾음 & 주차 가능
    std_msgs::Bool isComplete;          // 주차 완료
    std_msgs::Bool isReturnRail;        // 다시 주행하기 위해 돌아감
    std_msgs::Bool isReturned;          // 원래 주행로 복귀함
    
    /* Publisher Method */
    void publish(int gear, int velocity, float steer);

    /* Subscriber Callback */
    void subIMUCallback(const std_msgs::Float32 &subIMUMsgs);
    void subWaypointYCallback(const std_msgs::Float32 &subWaypointYMsgs);
    void subParkingFrontCallback(const std_msgs::Bool &subParkingMsgs);
    void subParkingRearCallback(const std_msgs::Bool &subParkingAreaMsgs);
    void subCompleteCallback(const std_msgs::Bool &subEndParkingMsgs);
    void subReturnToRailCallback(const std_msgs::Bool &subReverseMsgs);
    void subReturnedCallback(const std_msgs::Bool &subEndReverseMsgs);
};
