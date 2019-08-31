#include "AP_IMU.h"
#include <serial/serial.h>
#include <iostream>

serial::Serial serialPort;

bool validimu()
{
    if(serialPort.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
        return true;
    }
    else
        return false;
}

bool connectIMU(std::string port_name, int baudrate)
{
    if(validimu())
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
    connectIMU(port_name, baudrate);
    std::cout << "do?" << std::endl;
    IMU_flag = 0;
    IMUProcessing();
}

void IMU:: IMUProcessing(void)    //라이다 알고리즘
{
    std::string str2;
    if(serialPort.available())
    {
        serialPort.readline(IMUresult);
        std::cout<<"IMU: "<< IMUresult << std::endl;
        IMUresult.erase(0,18);
        str2 = IMUresult.substr(0,9);
        IMUresult.erase();

        if(IMU_flag != 2)
        {
            init_angle_z = std::atof(str2.c_str());
            IMU_flag++;
        }
        else
            angle_z = std::atof(str2.c_str())- init_angle_z;
    }
}
