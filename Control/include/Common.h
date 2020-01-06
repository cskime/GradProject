#include <iostream>
#include <ros/ros.h>

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
