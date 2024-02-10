#include "markers.h"

namespace sdpo_ratf_ros_gui{
    boxMarkers::boxMarkers(){
        getPositions();

        tf2::Quaternion quat;
    
        for(int i=0; i < 4;  i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = 0.09;
            marker.scale.y = 0.06;
            marker.scale.z = 0.05;
            marker.color.a = 1.0; // Don't forget to set the alpha!

            markers_array.markers.push_back(marker);
        }

        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/boxes_marker", 1);

        boxes_sub = nh.subscribe("/boxes_position", 1, &boxMarkers::cbBoxes, this);

    }

    boxMarkers::~boxMarkers(){

    }

    void boxMarkers::cbBoxes(const std_msgs::Int8MultiArray::ConstPtr& msg){
        std::vector<int> aux;

        for(int i =0; i< msg->data.size(); i++){
            aux.push_back(msg->data.at(i));
        }

        publishMarkers(aux, "BGRB");
    }

    void boxMarkers::getPositions(){
        XmlRpc::XmlRpcValue points_list;
        try {   // always define matrix with decimal points (can cause problems)
            nh.getParam("points", points_list);
            ROS_ASSERT(points_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

            for (int j = 0; j < points_list.size(); j++) {
                try {
                std::ostringstream ox,oy;
                std::vector<float> aux;

                ox << points_list[j]["x"];
                oy << points_list[j]["y"];

                aux.push_back(std::stof(ox.str()));
                aux.push_back(std::stof(oy.str()));
                points.push_back(aux);

                } catch (XmlRpc::XmlRpcException& e) {
                ROS_ERROR("[sdpo_ratf_ros_gui] Exception: %s",
                    e.getMessage());
                } catch (...) {
                throw;
                }
            }

        } catch (XmlRpc::XmlRpcException& e) {
            ROS_ERROR("[sdpo_ratf_ros_gui] Exception: %s",
                    e.getMessage());
        } catch (...) {
            throw std::runtime_error(
                "[markers.cpp] boxMarkers::getPositions: "
                "something went wrong when reading positions from the ROS param server");
        }
    }

    void boxMarkers::publishMarkers(std::vector<int> pos, std::string colors){
        tf2::Quaternion quat;
        int orientation = 0;

        for (int i=0; i < pos.size(); i++){
            markers_array.markers.at(i).pose.position.x = 0.001*points[pos.at(i)-50][0];
            markers_array.markers.at(i).pose.position.y= 0.001*points[pos.at(i)-50][1]-0.01;
            markers_array.markers.at(i).pose.position.z = 0;

            if(pos.at(i) > 53 && pos.at(i) < 62){
                orientation = 90;
            }
            else{
                orientation = 0;
            }

            quat.setRPY(0,0,orientation*M_PI/180);
            markers_array.markers.at(i).pose.orientation = tf2::toMsg(quat);

            switch(colors.at(i)){
                case 'B' : case 'b':{
                    markers_array.markers.at(i).color.r = 0.0;
                    markers_array.markers.at(i).color.g = 0.0;
                    markers_array.markers.at(i).color.b = 1.0;
                    break;
                }
                case 'G' : case 'g':{
                    markers_array.markers.at(i).color.r = 0.0;
                    markers_array.markers.at(i).color.g = 1.0;
                    markers_array.markers.at(i).color.b = 0.0;
                    break;
                }
                case 'R' : case 'r':{
                    markers_array.markers.at(i).color.r = 1.0;
                    markers_array.markers.at(i).color.g = 0.0;
                    markers_array.markers.at(i).color.b = 0.0;
                    break;
                }
            }
        }

        marker_pub.publish(markers_array);
    };
}