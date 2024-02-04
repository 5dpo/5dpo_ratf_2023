#include <stdlib.h>
#include <ros/ros.h>
#include "Location.h"
#include "Graph.h"
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

Graph<Location> create_graph(XmlRpc::XmlRpcValue nodes_list,XmlRpc::XmlRpcValue edges_list);


Robot rob;
//tf::TransformListener tf_listen_;

std::string map_frame_id_;
std::string base_frame_id_;

ros::Subscriber switch_sub;
ros::ServiceClient magnet_client;

void CbSwitchState(const std_msgs::Bool::ConstPtr& msg);
void use_magnet(bool logic_level);
