#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

class ControllerMessageManager {
private:    /* common */
    ros::NodeHandle node_;
    
    /* publish */
    ros::Publisher pub_gear;        // 전진(D), 후진(R), 정지(P)
    ros::Publisher pub_velocity;    // speed = 150
    ros::Publisher pub_steer;       // steering calculated with wayX
    
    /* subscribe */
    ros::Subscriber sub_heading;
    ros::Subscriber sub_waypointX;
    ros::Subscriber sub_isFindSign;
    ros::Subscriber sub_isAvailable;
    
    std_msgs::Float32 heading;        // IMU heading
    std_msgs::Float32 waypointX;      // Camera waypointX
    std_msgs::Bool isFindSign;        // Camera find parking sign
    std_msgs::Bool isAvailable;       // Camera check is park available
    
    /* Subscriber Callback */
    void subIMUCallback(const std_msgs::Float32 &subIMUMsgs);
    void subWaypointXCallback(const std_msgs::Float32 &subWaypointXMsgs);
    void subFindSignCallback(const std_msgs::Bool &subFindSignMsgs);
    void subAvailableCallback(const std_msgs::Bool &subAvailableMsgs);
    
public:
    
    void publish(int gear, int velocity, float steer);
    float getHeading();
    float getWayPointX();
    bool isFindParkingSign();
    bool isParkingAvailable();
};
