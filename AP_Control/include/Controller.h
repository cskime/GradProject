#include <iostream>

#include "ControllerMessageManager.h"
#include "Common.h"

class Controller {
    ControlMessageManager message;
    SteerQueue steerPool(1000);
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
    Controller();
    void parking();
}


