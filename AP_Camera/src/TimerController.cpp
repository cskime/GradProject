#include "Timer.h"

/* Using Topics
 * 1. geometry_msgs/Point : point msgs
 * 2. std_msgs/Bool : boolean msgs
 */

//헤더에서 ros, topic 관련 라이브러리 추가
//#include <geometry_msgs/Point.h>
//#include <std_msgs/Bool.h>


/* Timer class
 */
SC_Timer::SC_Timer()
{
    timer = node_.createTimer(ros::Duration(0.1), &SC_Timer::timer_callback, this);
    isCounted = false;
    sec_start = 0;
    sec_flag = 0;
    sec_count = 100;
}

void SC_Timer::set_timer(int th_sec)
{
    if(sec_flag==0)
    {
        if(sec_start == 0)
        {
            sec_start = sec_count;
            sec_flag = 1;
        }
    }
    doing_t =th_sec;

}

void SC_Timer::counting(void)
{
    if(sec_flag !=0)
    {
        if(sec_count != sec_start)
        {
            sec_flag = sec_count - sec_start;
        }
        if(sec_flag >= doing_t)
        {
            isCounted = false;
            sec_flag = 0;
            sec_start = 0;
        }
        else
            isCounted = true;
        //cout << "sec_flag: " << sec_flag << "   sec_count: " << sec_count << "  sec_start: "<< sec_start<< endl;
    }
}

void SC_Timer::exit()
{
    isCounted = false;
    sec_flag = 999999;
}

void SC_Timer::timer_callback(const ros::TimerEvent& )
{
    //std::cout << "Crosswalk stop line check.." << std::endl;
    sec_count++;

}
