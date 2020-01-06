#include "CameraController.h"

CameraManager::CameraManager()
{
    sideCamera = new SideCamera();
    frontCamera = new FrontCamera();
}

void CameraManager::process() {
    frontCamera->calculateWayPoint();
    sideCamera->findParkingArea();
    sideCamera->findParkingSign();
}

void CameraController::Processing()
{
    m_front = rawImagefront;
    m_side = rawImageside;

    frontProcessing();
    sideProcessing();
}

void CameraManager::sideProcessing()
{
    findSign();
    findParkingArea();
}

void CameraManager::findSign()
{
    // Migrated
}
