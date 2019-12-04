#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>
#include <ctime>


#define PI 3.141592


using namespace cv;
using namespace std;
using namespace ros;


class SC_Timer
{
    int sec_start;
    int sec_flag;
    int sec_count;
    int doing_t;

    ros::NodeHandle node_;
    ros::Timer timer;

    // MSG.Cross : find stopline(true/false)

public:
    SC_Timer();
    void timer_callback(const ros::TimerEvent& );
    void set_timer(int start_t);
    void counting(void);
    void exit(void);
    bool isCounted;

};
