#include "IMU.h"

bool IMU::isValid()
{
    if(serialPort.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
        return true;
    }
    else
        return false;
}

bool IMU::connect(std::string port_name, int baudrate)
{
    if(isValid())
        return false;
    try
    {
        serialPort.setPort(port_name);
        serialPort.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialPort.setTimeout(to);
        serialPort.open();
        serialPort.write("sp = 10"); // Period (ms)
        serialPort.write("ss = 4"); // Data type (Acc, RotRate, Angle) = 7
        ROS_INFO("Enable to open port ");
        return true;
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return false;
    }
}

IMU::IMU(std::string port_name, int baudrate)
{
    if (connect(port_name, baudrate))
    {
        flag = 0;
        initAngle = 0;
        heading = 0;
    }
}

float IMU::calculateHeading() {
    if(serialPort.available())
    {
        std::string headingStr;
        std::string response;
        
        serialPort.readline(response);
//        std::cout<<"IMU: "<< response << std::endl;
        response.erase(0,18);
        headingStr = response.substr(0,9);
        response.erase();

        if(flag != 2)
        {
            initAngle = std::atof(headingStr.c_str());
            flag++;
        }
        else
            heading = std::atof(headingStr.c_str()) - initAngle;
    }
    else {
        heading = 0;
    }
    return heading;
}
