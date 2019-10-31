#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

class ControllerMessageManager {
    
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
    ros::Subscriber sub_isReturned;
    
    std_msgs::Float32 heading;          // IMU heading
    std_msgs::Float32 waypointY;        // waypoint
    std_msgs::Bool isAvailableFront;    // 전면주차공간 찾음 & 주차 가능
    std_msgs::Bool isAvailableRear;     // 후면주차공간 찾음 & 주차 가능
    std_msgs::Bool isCompleteMsg;          // 주차 완료
    std_msgs::Bool isReturnedMsg;          // 원래 주행차로 복귀
    
    /* Subscriber Callback */
    void subIMUCallback(const std_msgs::Float32 &subIMUMsgs);
    void subWaypointYCallback(const std_msgs::Float32 &subWaypointYMsgs);
    void subParkingFrontCallback(const std_msgs::Bool &subParkingMsgs);
    void subParkingRearCallback(const std_msgs::Bool &subParkingAreaMsgs);
    void subCompleteCallback(const std_msgs::Bool &subEndParkingMsgs);
    void subReturnedCallback(const std_msgs::Bool &subEndReverseMsgs);
    
public:
    
    void publish(int gear, int velocity, float steer);
    bool isValidFront();
    bool isValidRear();
    bool isComplete();
    bool isReturned();
};
