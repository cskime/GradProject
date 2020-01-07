#include <iostream>

#include "ControllerMessageManager.h"
#include "Common.h"

class Controller {
    ControlMessageManager *msgManager;
    SteerQueue steerPool(1000);
    Timer timer[10];
    
    enum Gear {
        DRIVE = 0, REAR = 1, PARK = 2
    };
    Gear gear;
    int velocity;
    
    /* State Pattern */
    enum ParkingStates {
        SEARCH, PARKING_FRONT, COMPLETE, RETURN
    };
    ParkingStates state;
    void changeState(ParkingStates state);
    
    /* State Methods */
    short calculateSteer();
    void searchArea();
    void parkingFront();
    void parkingRear();
    void complete();
    void returnRail();
public:
    Controller();
    ~Controller() {
        delete msgManager;
    }
    void parking();
}


