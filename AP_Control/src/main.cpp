#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <fstream>
#include <queue>

#include "AP_Control.h"

using namespace cv;
using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control");

    MSGs msgs;
    CONTROL control;
    TIMER controlTimer[10]; // state 번호에 맞는 index의 timer 사용
    SteerList stl(1000);    // space

    int state = 0;


    // init
    int drivingSpeed = 140;
    int parkingSpeed = 120;

    while(ros::ok())
    {
        /* Parking State
         * 0: 주차구역이 아님
         * 1: 주차구역인데 주차할 수 없음
         * 2: 주차가능. 주차중
         * 3: 주차 완료
         * 4: 원래 차선으로 돌아감
         * 5: 차선 복귀 후 전진
         * 6: 후진주차구역 찾기
         * 7:
         */
        if(!msgs.isParkingArea) {
            state = 0;
        } else if(!msgs.isParkingAvailable) {
            state = 1;
        } else if(!msgs.endParking) {
            state = 2;
        } else if(!msgs.pReverse) {
            state = 3;
        } else if(!msgs.endReverse) {
            state = 4;
        } else {
            state = 5;
        }

        switch (state)
        {
        case 0:
            control.gear = 0;
            control.speed = drivingSpeed;
            control.steer = control.steer_LaneControl(msgs.waypointY);

            if(stl.isFull()) {
                stl.Dequeue();
            }
            stl.Enqueue(control.steer);

            cout << "직선주행: Steer " << control.steer << endl;
            break;

        case 1:
            control.gear = 0;
            control.speed = drivingSpeed;
            control.steer = control.steer_LaneControl(msgs.waypointY);;

            if(stl.isFull()) {
                stl.Dequeue();
            }
            stl.Enqueue(control.steer);

            cout << "주차구역 탐색: Steer " << control.steer << endl;
            break;

        case 2:
            // 7초동안 주차동작
            controlTimer[2].counting(70);  // 0.1sec/count
            if(!controlTimer[2].isCounted)
            {
                control.gear = 0;
                control.speed = parkingSpeed;
                control.steer = 45.0;    // full steering to right
                // control.steer = control.steer_LaneControl(msgs.waypointY);

                // 후진할 때 사용기기 위해 큐에 저장해둠
                if(stl.isFull()) {
                    stl.Dequeue();
                }
                stl.Enqueue(control.steer);

                cout << "주차중: Steer " << control.steer << endl;
                break;
            }
            else
            {
                msgs.endParking = true;
                controlTimer[2].close();
                break;
            }

//            control.gear = 0;
//            control.speed = 130;
//            control.steer = 45.0;    // full steering to right
//            // control.steer = control.steer_LaneControl(msgs.waypointY);

//            // 후진할 때 사용기기 위해 큐에 저장해둠
//            if(stl.isFull()) {
//                stl.Dequeue();
//            }
//            stl.Enqueue(control.steer);

//            cout << "주차중: Steer " << control.steer << endl;
//            break;
        case 3:
            // 주차 완료 후 3초간 대기
            controlTimer[3].counting(30);

            if(!controlTimer[3].isCounted)
            {
                control.gear = 2;
                control.speed = 0;
                control.steer = 0.0;
                // control.steer = control.steer_LaneControl(msgs.waypointY);

                cout << "주차완료" << endl;
                break;
            }
            else
            {
                msgs.pReverse = true;
                controlTimer[3].close();
                break;
            }
        case 4:
            if(!stl.isEmpty())
            {
                control.gear = 1;
                control.speed = parkingSpeed;
                //control.steer = control.steer_LaneControl(msgs.waypointY);

                control.steer = stl.Pop();
                cout << "후진: Steer " << control.steer << endl;
                break;
            }
            else
            {
                msgs.endReverse = true;
                break;
            }
        case 5:
            control.gear = 0;
            control.speed = drivingSpeed;
            control.steer = control.steer_LaneControl(msgs.waypointY);;

            if(stl.isFull()) {
                stl.Dequeue();
            }
            stl.Enqueue(control.steer);

            cout << "후진완료. 다시 주행합니다: Steer " << control.steer << endl;
            break;
        }

        msgs.gear.data = control.gear;
        msgs.speed.data = control.speed;
        msgs.steer.data = control.steer;

        msgs.pub_gear.publish(msgs.gear);
        msgs.pub_speed.publish(msgs.speed);
        msgs.pub_steer.publish(msgs.steer);

        //cout << "Arduino >> Gear : " << msgs.testGear << ", Speed : " << msgs.testSpeed << ", Steer : " << msgs.testSteer << endl;

        usleep(31000);
        ros::spinOnce();
    }
}
