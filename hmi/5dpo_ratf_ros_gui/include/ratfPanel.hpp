#ifndef ratf_panel_H_
#define ratf_panel_H_

#include <ros/ros.h>
#include <rviz/panel.h>
#include <string>
#include <cmath>
#define _USE_MATH_DEFINES
/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_ratf_panel.h>

// Other ROS dependencies
#include <std_msgs/Int16.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <XmlRpcException.h>

namespace sdpo_ratf_ros_gui
{   
    class ratfPanel : public rviz::Panel
    {
        Q_OBJECT
        public:
            ratfPanel(QWidget * parent = 0);

        private Q_SLOTS:
            void forceStart();
            void forceSwitch();

        protected:
            // UI pointer
            std::shared_ptr<Ui::rviz_panel> ui_;

    };
} // namespace ratf_panel

#endif