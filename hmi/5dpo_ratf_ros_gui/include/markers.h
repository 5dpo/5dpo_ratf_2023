#pragma once

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcException.h>
//#include "sdpo_ratf_ros_path_planning/boxes_info.h"

namespace sdpo_ratf_ros_gui {

    class boxMarkers{
      private:
        std::vector<std::vector<float>> points; 

        ros::Subscriber boxes_sub;
        ros::Publisher marker_pub;
        ros::NodeHandle nh;
        visualization_msgs::MarkerArray markers_array;

      public:
        boxMarkers();
        ~boxMarkers();

        void getPositions();
        void publishMarkers(std::vector<int> pos, std::string colors);

        void cbBoxes(const std_msgs::Int8MultiArray::ConstPtr& msg);
    };

} // namespace sdpo_ratf_ros_localization
