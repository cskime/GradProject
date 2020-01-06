#include "Common.h"

SteerQueue::SteerQueue(int size)
{
    this->size = size;
    list = new double[size];

    for(int i=0; i < size; i++)
        list[i] = 0.0f;

    head = tail = 0;
}

SteerQueue::~SteerQueue()
{
    delete[] list;
}

void SteerQueue::Enqueue(double num)
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

void SteerQueue::Dequeue()
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

double SteerQueue::Pop()
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
bool SteerQueue::isEmpty()
{
    if (head == tail) return true;
    else return false;
}

// is queue full
bool SteerQueue::isFull()
{
    if (head == ((tail + 1) % size)) return true;
    else return false;
}

void SteerQueue::show()
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
