#include "Controller.h"

using namespace std;

Controller::Controller()
{
    velocity = 150;
    state = SEARCH;
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
    if (steerPool.isFull())
        steerPool.Dequeue();
    else
        steerPool.Enqueue(steer);
    message.publish(0, velocity, steer);
    
    if (message.isValidFront()) {
        changeState(PARKING_FRONT);
        return;
    }
    
    if (message.isValidRear()) {
        changeState(PARKING_REAR);
        return;
    }
}

void Controller::parkingFront() {
    timer[PARKING_FRONT].counting(70);  // 7sec moving
    
    if (!timer[PARKING_FRONT].isCounted || !message.isComplete()) {
        float steer = 45;
        if (steerPool.isFull())
            steerPool.Dequeue();
        else
            steerPool.Enqueue(steer);
        message.publish(0, velocity - 20, steer);
    } else {
        timer[PARKING_FRONT].close();
        changeState(COMPLETE);
    }
}

void Controller::parkingRear() {
//    messagePool->publish(0, velocity, );
}

void Controller::complete() {
    timer[COMPLETE].counting(30);   // 3sec wait after complete
    
    if (!timer[COMPLETE].isCounted) {
        message.publish(0, 0, 0);
    } else {
        timer[COMPLETE].close();
        changeState(RETURN);
    }
}

void Controller::returnRail() {
    if (!message.isReturned()) {
        float steer = steerPool.Pop();
        message.publish(1, velocity - 20, steer);
    } else {
        changeState(SEARCH);
    }
}

void Controller::changeState(ParkingStates state) {
    this->state = state;
}

short Controller::calculateSteer() {
    float pixel_y = message.waypointY.data;
    float k = 0.35;    // gain k = 0.19 =: 22.5
    int x_th = -10;   // 치우침 보정
    short steer = k*(pixel_y+x_th-320);     //atan(-p_y/p_x) * 180/3.14 * 2000/22.5;
    // 2000, 22.5 를 자동차에 맞게 수정

    if(steer > 45)  steer = 45;
    if(steer < -45) steer = -45;

    return steer;
}
