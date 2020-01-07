#include "Controller.h"

using namespace std;

Controller::Controller()
{
    msgManager = new ControllerMessageManager();
    velocity = 150;
    state = SEARCH;
    gear = DRIVE;
}

void Controller::parking() {
    switch (state) {
        case SEARCH:
            searchArea();
            break;
        case PARKING_FRONT:
            parkingFront();
            break;
        case PARKING_REAR:
            parkingRear();
            break;
        case COMPLETE:
            complete();
            break;
        case RETURN:
            returnRail();
            break;
    }
}

void Controller::searchArea() {
    float steer = calculateSteer();
    gear = DRIVE;
    
    if (steerPool.isFull()) {
        steerPool.Dequeue();
    } else {
        steerPool.Enqueue(steer);
    }
    msgManager->publish(gear, velocity, steer);
    
    bool isFindSign = msgManager->isFindParkingSign();
    bool isAvailable = msgManager->isParkingAvailable();
    
    if (isFindSign && isAvailable) {
        changeState(PARKING_FRONT);
    }
}

void Controller::parkingFront() {
    timer[PARKING_FRONT].counting(70);  // 7sec moving
    gear = DRIVE;
    
    if (!timer[PARKING_FRONT].isCounted) {
        float steer = 45;
        if (steerPool.isFull()) {
            steerPool.Dequeue();
        } else {
            steerPool.Enqueue(steer);
        }
        message.publish(gear, velocity - 20, steer);
    } else {
        timer[PARKING_FRONT].close();
        changeState(COMPLETE);
    }
}

void Controller::complete() {
    timer[COMPLETE].counting(30);   // 3sec wait after complete
    gear = PARK;
    
    if (!timer[COMPLETE].isCounted) {
        msgManager->publish(gear, 0, 0);
        message.publish(gear, 0, 0);
    } else {
        timer[COMPLETE].close();
        changeState(RETURN);
    }
}

void Controller::returnRail() {
    timer[RETURN].counting(70);
    gera = REAR;
    
    if (!timer[RETURN].isCounted) {
        float steer = steerPool.Pop();
        message.publish(gear, velocity - 20, steer);
    } else {
        timer[RETURN].close();
        changeState(SEARCH);
    }
}

void Controller::changeState(ParkingStates state) {
    this->state = state;
}

short Controller::calculateSteer() {
    float pixel_y = msgManager->getWayPointX();
    float k = 0.35;    // gain k = 0.19 =: 22.5
    int x_th = -10;   // 치우침 보정
    short steer = k*(pixel_y+x_th-320);     //atan(-p_y/p_x) * 180/3.14 * 2000/22.5;
    // 2000, 22.5 를 자동차에 맞게 수정

    if(steer > 45)  steer = 45;
    if(steer < -45) steer = -45;

    return steer;
}
