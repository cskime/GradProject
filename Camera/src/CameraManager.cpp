#include "CameraManager.h"

CameraManager::CameraManager()
{
    msgManager = new CameraMessageManager();
    Mat sideImage = msgManager->getSideImage();
    Mat frontImage = msgManager->getFrontImage();
    
    sideCamera = new SideCamera(sideImage);
    frontCamera = new FrontCamera(frontImage);
}

void CameraManager::process() {
    float wayPointX = frontCamera->calculateWayPoint();
    msgManager->publishWay(wayPointX);
    
    bool canParking = sideCamera->findParkingArea();
    msgManager->publishSign(isFindSign);
    
    bool isFindSign = sideCamera->findParkingSign();
    msgManager->publishParkingAvailable(canParking);
}
