#include "Robot.h"
#include <eigen3/Eigen/Dense>

    Robot::Robot(double k_speed, double k_corr, double k_w, std::string udp_ip, int udp_port,double kd_v, double kd_w){
        this->position= Location(21, -0.6950+0.05, -0.3550);
        this->theta = 1.5708;
        this->k_speed=k_speed;
        this->k_corr=k_corr;
        this->k_w=k_w;
        this->pid_x=PID(k_corr,kd_v,0,k_speed);
        this->pid_y=PID(k_corr,kd_v,0,k_speed);
        this->pid_theta=PID(k_w,kd_w,0,0.8); //0.6
        comms = new::udp::UdpClient(udp_ip, udp_port);
    }

    void Robot::follow(Line l){
        Location intr=l.dist2Line(this->position);
        Eigen::Vector2d r2end(l.getXf()-position.getX(),l.getYf()-position.getY());
        r2end.normalize();
        Eigen::Vector2d r = l.getVec();
        r.normalize();
        if (r.dot(r2end)<-0.9){
            r=-1*r;
        }
        Eigen::Vector2d res;
        double error_x=intr.getX()-this->position.getX();
        double error_y=intr.getY()-this->position.getY();
        res.x()=r.x()*k_speed*0.8+this->pid_x.compute(error_x); //if necessary pass k_speed as a parameter
        res.y()=r.y()*k_speed*0.8+this->pid_y.compute(error_y);
        res.normalize();
        res=res*k_speed;
        v_x=res.x();
        v_y=res.y();
        this->transformToRobRef();
    
    }

    void Robot::rotate(){
        this->omega=this->pid_theta.compute(normalizeAngle(theta_ref-theta));
    }

    void Robot::transformToRobRef(){
        this->V = (this->v_x*cos(this->theta)+this->v_y*sin(this->theta));
        this->Vn = (-this->v_x*sin(this->theta)+this->v_y*cos(this->theta));
    }

    void Robot::updatePosition(double x, double y, double theta){
        this->position.setLocation(x,y);
        this->theta=this->normalizeAngle(theta);
    }
    void  Robot::create_path(int loc_id){
        Location closest = this->closest_id();
        path=this->graph.dijkstraShortestPath(closest,this->graph.findVertexById(loc_id)->getInfo()); // 59 must be substituted by the desired location
         for (auto i: path){
                std::cout << i.getID() << " ";
            }
            ROS_INFO("PATH");
            line_list= generateLines(path);
            for (auto i: line_list){
                std::cout <<"X: " << i.getXi() << ' ' << i.getXf() << "\n";
                std::cout <<"Y: "<< i.getYi() << ' ' << i.getYf() << "  next\n \n";}
            ROS_INFO("Not Trimmed");
            trimmed_list= trimLines(line_list);
            for (auto i: trimmed_list){
                std::cout <<"X: " << i.getXi() << ' ' << i.getXf() << "\n";
                std::cout <<"Y: "<< i.getYi() << ' ' << i.getYf() << "  next\n \n";}
            ROS_INFO("Trimmed");
        this->theta_ref=normalizeAngle(trimmed_list.back().orientation);    
        std::cout << "THETA_REF  " << this->theta_ref<< std::endl;
    }

    Location Robot::closest_id(){
        Location closest = this->graph.findClosestLocation(this->position);
        return closest;
    }

    float Robot::normalizeAngle(float angle){
    float new_angle;
    int num;
    
    if(angle > M_PI){
        num = angle/M_PI;
        new_angle = angle-(num*2*M_PI);
    }
    else if (angle < -M_PI){
        num = angle/M_PI;
        new_angle = angle - (num*2*M_PI);
    }
    else{
        new_angle = angle;
    }

    return new_angle;
}
    