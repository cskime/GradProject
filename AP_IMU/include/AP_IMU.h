#include <ros/ros.h>

class IMU {
     public:
        IMU(std::__cxx11::string port_name, int baudrate);                // 생성자. 초기인풋을 정해준다.
        float init_angle_z;
        float angle_z;

        void IMUProcessing(void);
    private:
        int IMU_flag;
        ros::NodeHandle node_;
        std::string IMUresult;      //IMU 데이터값
};
