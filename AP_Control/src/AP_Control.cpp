#include "AP_Control.h"

using namespace std;

short CONTROL::steer_LaneControl(float pixel_y)
{
    float k = 0.35;    // gain k = 0.19 =: 22.5
    int x_th = -10;   // 치우침 보정
    short steer = k*(pixel_y+x_th-320);     //atan(-p_y/p_x) * 180/3.14 * 2000/22.5;
    // 2000, 22.5 를 자동차에 맞게 수정

    if(steer > 45)  steer = 45;
    if(steer < -45) steer = -45;

    return steer;
}

SteerList::SteerList(int size)
{
    this->size = size;
    list = new double[size];

    for(int i=0; i < size; i++)
        list[i] = 0.0f;

    head = tail = 0;
}

SteerList::~SteerList()
{
    delete[] list;
}

void SteerList::Enqueue(double num)
{
    if(isFull())
    {
        cout << "Queue Overflow" << endl;
    }
    else
    {
        list[tail] = num;
        tail = (tail + 1) % size;
    }
}

void SteerList::Dequeue()
{
    if (isEmpty())
    {
        cout << "Queue Empty" << endl;
    }

    else
    {
        head = (head + 1) % size;
    }
}

double SteerList::Pop()
{
    if (isEmpty())
    {
        cout << "Stack Empty" << endl;
        return 0;
    }

    else
    {
        if (tail < 1) 
            tail += size;
        return list[--tail];
    }
}

// is queue empty
bool SteerList::isEmpty()
{
    if ((head == tail)) return true;
    else return false;
}

// is queue full
bool SteerList::isFull()
{
    if (head == ((tail + 1) % size)) return true;
    else return false;
}

void SteerList::show()
{
    //cout << "Head : " << head << ", Head Value : " << list[head] << endl;
    //cout << "Tail : " << tail << ", Tail Value : " << list[tail] << endl;
    for (int i = 0; i < size; i++)
    {
        cout << list[i] << " ";
        if (i % 10 == 9) cout << endl;
    }
    cout << endl;
}

TIMER::TIMER()
{
    timer = node_.createTimer(ros::Duration(0.1), &TIMER::timer_callback, this);
    isCounted = false;
    isOpen = true;
}

void TIMER::close()
{
    isOpen = false;
    isCounted = false;
    sec_flag = 99999;
}

void TIMER::counting(int th_sec)
{
    if(isOpen)
    {
        // 최초 counting
        if(sec_flag == 0)
        {
            if(sec_start == 0)
            {
                sec_start = sec_count;
                sec_flag = 1;
            }
        }
        else if(sec_count != sec_start)
        {
            sec_flag = sec_count - sec_start;
        }

        cout << "sec_start : " << sec_start << ", sec_count : " << sec_count << ", sec_flag : " << sec_flag << endl;

        // 측정된 sec_flag로 계산
        if(sec_flag >= th_sec)
        {
            isCounted = true;
        }
        else
            isCounted = false;
    }
}

void TIMER::timer_callback(const ros::TimerEvent& )
{
    sec_count++;
}
