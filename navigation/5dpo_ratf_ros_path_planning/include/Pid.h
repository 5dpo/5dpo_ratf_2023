#include <ros/ros.h>

class PID{
    ros::Time lastTimeStamp;
    double kp, ki, kd;
    double integralError=0;
    double saturation;
    double lastError=0;
    bool initialized=false;
    
    
public:
    PID(){};
    PID(double kp, double ki, double kd,double saturation);
    double compute(double error);
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);
    double getKp();
    double getKi();
    double getKd();   
};