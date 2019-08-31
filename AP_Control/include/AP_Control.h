#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>

#include "AP_Message.h"

class CONTROL
{
public:
    CONTROL(){
        gear = 2;
        speed = 0;
        steer = 0;
    }

    int gear;
    int speed;
    float steer;

    short steer_LaneControl(float pixel_y);
};

// Steer save
class SteerList {
    double *list;
    int size;
    int head;
    int tail;
public:
    SteerList(int size);
    ~SteerList();

    void Enqueue(double num);
    void Dequeue();
    double Pop();
    bool isEmpty();
    bool isFull();

    void show();
};

// Timer class
class TIMER
{
    int sec_start = 0;
    int sec_flag = 0;
    int sec_count = 0;
    bool isOpen;

    ros::NodeHandle node_;
    ros::Timer timer;

    // MSG.Cross : find stopline(true/false)

public:
    TIMER();
    void timer_callback(const ros::TimerEvent& );
    void counting(int th_sec);
    void close();
    bool isCounted;

    static bool endParking;
    static bool endStaying;
    static bool endReversing;
};
