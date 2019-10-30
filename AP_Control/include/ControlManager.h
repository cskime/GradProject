#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>

#include "ControlMessageManager.h"

class ControlManager {
    ControlMessageManager message;
    SteerQueue steerQueue(1000);
    Timer timer[10];
    
    int velocity;
    
    short calculateSteer();
    
    /* State Control */
    enum ParkingStates {
        SEARCH = 0, PARKING_FRONT, PARKING_REAR, COMPLETE, RETURN
    };
    ParkingStates state;
    void searchArea();
    void parkingFront();
    void parkingRear();
    void complete();
    void returnRail();
    void changeState(ParkingStates state);
public:
    ParkingController();
    void parking();
}

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

// Steer save
class SteerQueue {
    double *list;
    int size;
    int head;
    int tail;
public:
    SteerQueue(int size);
    ~SteerQueue();

    void Enqueue(double num);
    void Dequeue();
    double Pop();
    bool isEmpty();
    bool isFull();

    void show();
};


