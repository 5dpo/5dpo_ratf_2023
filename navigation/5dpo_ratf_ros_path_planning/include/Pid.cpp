#include "Pid.h"

PID::PID(double kp,double ki,double kd,double saturation) : kp(kp) , ki(ki),kd(kd), saturation(saturation)  {
}

double PID::compute(double error){
    double u;
    ros::Time new_timestamp = ros::Time::now();
    if (!initialized){
        initialized=true;
        u=kp*error;
    }
    else{
        double dt = new_timestamp.toSec()-this->lastTimeStamp.toSec();
        
        this->integralError += error*dt;

        double derivativeError = (error-this->lastError)/dt; //TODO Derivative Filter
        
        u = kp*error + this->ki*this->integralError + this->kd*derivativeError;

        if (u>this->saturation) //anti-windup
        {
            u=this->saturation;
            this->integralError-=error*dt;
        }
        else if (u<-this->saturation) //anti-windup
        {
            u=-this->saturation;
            this->integralError-=error*dt;
        }

        this->lastError=error; 
        }
    this->lastTimeStamp=new_timestamp;

    return u;
    
}


void PID::setKp(double kp){
    this->kp=kp;
}
void PID::setKi(double ki){
    this->ki=ki;
}
void PID::setKd(double kd){
    this->kd=kd;
}
double PID::getKp(){
    return this->kp;
}
double PID::getKi(){
    return this->ki;
}
double PID::getKd(){
    return this->kd;
}