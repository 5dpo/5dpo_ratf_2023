#include <stdlib.h>
#include <ros/ros.h>
#include "Location.h"
#include "Graph.h"
#include "Robot.h"
#include "path_planning.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include "sdpo_ratf_ros_path_planning/boxes_info.h"

#include <XmlRpcException.h>

double k_speed;
double k_correction;
double k_w, kd_w, kd_v;
double dist_tresh;
double box_dist;
double rotate_dist;
double slow_zone;
double speed_divisor;
bool mach_b;
std::string udp_srv_msg;

void CbSwitchState(const std_msgs::Bool::ConstPtr& msg){
    rob.switch_on = msg->data;
}

void getParameters(ros::NodeHandle nh){
    nh.getParam("k_speed", k_speed);
    nh.getParam("k_correction", k_correction);
    nh.getParam("k_w", k_w);
    nh.getParam("dist_tresh", dist_tresh);
    nh.getParam("box_dist", box_dist);
    nh.getParam("rotate_dist", rotate_dist);
    nh.getParam("kd_v", kd_v);
    nh.getParam("kd_w", kd_w);
    nh.getParam("slow_zone", slow_zone);
    nh.getParam("speed_divisor", speed_divisor);
    nh.getParam("mach_b", mach_b);
    //nh.getParam("box_order", srv_msg);
}

float normalizeAngle(float angle){
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sdpo_path_planning");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    tf::TransformListener tf_listen_;
    ROS_INFO("Launching Path Planning");
    ros::Publisher vel_pub, boxes_pub;
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    boxes_pub = nh.advertise<sdpo_ratf_ros_path_planning::boxes_info>("boxes_position",1);
    getParameters(nh_priv);
    
    // Coordinate Frame IDs
    nh_priv.param<string>("map_frame_id", map_frame_id_,
                                    "map");
    // ROS_INFO("[sdpo_ratf_ros_localization] Map frame ID: %s",
    //         map_frame_id_.c_str());

    nh_priv.param<string>("base_frame_id", base_frame_id_,
                                    "base_footprint");
    // ROS_INFO("[sdpo_ratf_ros_localization] Base footprint frame ID: %s",
    //         base_frame_id_.c_str());


    XmlRpc::XmlRpcValue nodes_list;
    XmlRpc::XmlRpcValue edges_list;
    std::string udp_server_ip;
    int udp_server_port;
    try {
        nh_priv.getParam("edges",edges_list);
        nh_priv.getParam("nodes",nodes_list);
        nh_priv.getParam("ip_address", udp_server_ip);
        nh_priv.getParam("port", udp_server_port);
    } catch (XmlRpc::XmlRpcException& e) {
        ROS_INFO("[sdpo_ratf_ros_path_planning] Exception: %s",
                e.getMessage().c_str());
    }

    
    switch_sub = nh.subscribe<std_msgs::Bool>("switch_state", 1, CbSwitchState);
    magnet_client = nh.serviceClient<std_srvs::SetBool>("set_solenoid_state");

    rob = Robot(k_speed,k_correction,k_w, udp_server_ip, udp_server_port,kd_v,kd_w);
    rob.man.mach_b=mach_b;
    rob.graph=create_graph(nodes_list,edges_list);
    geometry_msgs::Twist vel_msg;
    Line current_line;
    ros::Rate rate(50);
    tf_listen_.waitForTransform(map_frame_id_, base_frame_id_, ros::Time::now(),
                    ros::Duration(5.0));

        

    while (!rob.comms->is_connected){
        try{
            rob.comms->connect();
        }catch(const char *msg){
            std::cout << msg << std::endl;
            return 0;
        }
        
    }

    udp_srv_msg = "STOP";
    int retries = 2;

    while(!udp_srv_msg.compare("STOP")){
        try{
            rob.comms->sendMsg("IWP");
            udp_srv_msg = rob.comms->rcvMsg();

            if(udp_srv_msg.empty() && retries==0){
                std::cout << "UDP Server not responding" << std::endl;
                return 0; 
            }

            if(udp_srv_msg.empty()){
                udp_srv_msg = "STOP";
            }

            retries--;
            
        }catch(const char *msg){
            std::cout << msg << std::endl;
            return 0;
        }    
    }

    //string srv_msg = "BGRB";
    rob.comms->disconnect();
    std::cout << "BOX ORDER " << udp_srv_msg  << std::endl;
    rob.man.get_boxes(udp_srv_msg);
    sdpo_ratf_ros_path_planning::boxes_info boxes;
    for(int i=0; i<rob.man.box_list.size(); i++)
    {
        boxes.boxes_id.push_back(rob.man.box_list[i].location);
    }
    boxes.box_type = udp_srv_msg;

    rob.man.current_box = rob.man.findPriorityBox();

   
   rob.create_path(rob.man.box_list[rob.man.current_box].location);
   current_line=rob.trimmed_list[rob.line_state];
   Location last_loc= Location(0,0,0.1);


    while(ros::ok())
    {   
        tf::StampedTransform tf_base2map;
        try {
            tf_listen_.lookupTransform(map_frame_id_, base_frame_id_, ros::Time(0),
                tf_base2map);
        } catch (tf::TransformException& e) {
            throw std::runtime_error(e.what());
        }
        tf::Quaternion quat;
        tf::Vector3 pos;
        double yaw;
        pos = tf_base2map.getOrigin();
        quat = tf_base2map.getRotation();
        yaw = normalizeAngle(tf::getYaw(quat));
        rob.updatePosition(pos[0], pos[1], yaw);


        if (rob.following)
        {
            if(current_line.getDist2End(rob.position)<dist_tresh)
            {
                rob.line_state++; 
                if (rob.line_state < rob.trimmed_list.size())
                {
                    current_line=rob.trimmed_list[rob.line_state];
                }
                else 
                {
                    rob.following=false;   
                    std::cout << "Reached Goal  " << rob.current_box <<std::endl;
                    if (rob.current_box==4)
                    {
                        for (int rt=0; rt<10;rt++){
                            vel_msg.linear.x=0;
                            vel_msg.linear.y=0;
                            vel_msg.angular.z=0;
                            rob.following=false;
                            rob.can_rotate=false;
                            vel_pub.publish(vel_msg);
                            ros::spinOnce();
                            rate.sleep();
                            
                        }
                        break;
                    }         
                }               
            }
        }
        else
        {  
            Location box_location=rob.graph.findVertexById(rob.man.box_list[rob.man.current_box].location)->getInfo();
            Location box_destiny=rob.graph.findVertexById(rob.man.box_list[rob.man.current_box].destiny)->getInfo();
            if(rob.switch_on == true and (getDistBetweenLocations(rob.position,box_location) < dist_tresh))
            {                      
                rob.line_state=0; 
                rob.magnet=true;
                use_magnet(rob.magnet);
                
                bool calculated=rob.man.calculate_destinies(rob.man.current_box);
                rob.create_path(rob.man.box_list[rob.man.current_box].destiny);
                current_line=rob.trimmed_list[rob.line_state];
                rob.following=true;
                rob.can_rotate=false;
                last_loc=rob.graph.findVertexById(rob.man.box_list[rob.man.current_box].location)->getInfo();
                

            }
            else if(getDistBetweenLocations(rob.position,box_destiny)<dist_tresh)
            {   
                rob.line_state=0; 
                last_loc=rob.graph.findVertexById(rob.man.box_list[rob.man.current_box].destiny)->getInfo();
                rob.man.complete_operation();
                if (rob.man.current_box==4){ //finished
                    rob.magnet=false;
                    use_magnet(rob.magnet);       
                    rob.create_path(5);
                    rob.current_box=4;
                    current_line=rob.trimmed_list[rob.line_state];
                    rob.following=true;
                    rob.can_rotate=false;
                    rob.theta_ref*=-1;
                    //last_loc=rob.graph.findVertexById(rob.man.box_list[rob.man.current_box].destiny)->getInfo();
                    continue;            
                    //break;
                } 
                else{                 
                    rob.create_path(rob.man.box_list[rob.man.current_box].location);
                    current_line=rob.trimmed_list[rob.line_state];
                    for(int i=0; i<rob.man.box_list.size(); i++)
                    {
                        boxes.boxes_id[i]=rob.man.box_list[i].location;
                    }
                    rob.following=true;
                    rob.can_rotate=false;
                    rob.magnet=false;
                    use_magnet(rob.magnet);
                }
                
            }
        }

    /*Outputs*/
    if (rob.following)
    {
        rob.follow(current_line);
        if(rob.line_state +1 == rob.trimmed_list.size() && current_line.getDist2End(rob.position)<slow_zone)
        {
        if( std::abs(normalizeAngle(rob.theta_ref-rob.theta)) > 0.2)
        {
            rob.V=0;
            rob.Vn=0;
        }
        else
        {
            rob.V/=speed_divisor;
            rob.Vn/=speed_divisor;
        }
        }
        if (rob.can_rotate){
        rob.rotate();
        }
        else
        {
        if (getDistBetweenLocations(last_loc,rob.position)>rotate_dist){
            rob.can_rotate=true;
        }
        }
    }
    
    vel_msg.linear.x=rob.V;
    vel_msg.linear.y=rob.Vn;
    vel_msg.angular.z=rob.omega;
    vel_pub.publish(vel_msg);
    boxes_pub.publish(boxes);
        
    
    rate.sleep();
    ros::spinOnce();


    }
}



Graph<Location> create_graph(XmlRpc::XmlRpcValue nodes_list,XmlRpc::XmlRpcValue edges_list){
    Graph<Location> g;
    for (int32_t i = 0; i < nodes_list.size(); ++i) 
        {
        int id = int(nodes_list[i]["id"]);
        double x = double(nodes_list[i]["x"]);
        double y = double(nodes_list[i]["y"]);
        Location l=Location(id,double(x)*0.001,double(y)*0.001);
        bool success=g.addVertex(l);
        // if (success)
        //     std::cout << "id " << id << " x : " << double(x)*0.001 << " y : " << double(y)*0.001 << std::endl;
        // else
        //     std::cout << "failed at id " << id<< std::endl;
        }
        

    for (int32_t i = 0; i < edges_list.size(); ++i) 
        {
        int source = int(edges_list[i]["source"]);
        int target = int(edges_list[i]["target"]);
        bool success1=g.addEdge(g.findVertexById(source)->getInfo(),g.findVertexById(target)->getInfo());
        bool success2=g.addEdge(g.findVertexById(target)->getInfo(),g.findVertexById(source)->getInfo()); //bidirectional graph
        // if (success1 && success2)
        //     std::cout << "Edge from " << source << " to " << target <<  std::endl;
        // else
        //     std::cout << "FAILED AT "  << "Edge from " << source << " to " << target <<  std::endl;

         }
    return g;

}

void use_magnet(bool logic_level)
{
    std_srvs::SetBool srv_msg;
    srv_msg.request.data = logic_level;
    if(magnet_client.call(srv_msg)){
        //ROS_INFO("Magnet not active");
    }
    else{
        //ROS_ERROR("Magnet desactivation failed");
    }
}



