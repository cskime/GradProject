#include <iostream>
#include <serial/serial.h>

class IMU {
private:
    serial::Serial serialPort;
    
    int flag;
    float initAngle;
    float heading;
    
    bool connect(std::string port_name, int baudrate);
    bool isValid();
public:
    IMU(std::__cxx11::string port_name, int baudrate);
    float calculateHeading();
};
