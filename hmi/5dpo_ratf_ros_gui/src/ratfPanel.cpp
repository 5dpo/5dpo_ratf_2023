#include "ratfPanel.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(sdpo_ratf_ros_gui::ratfPanel, rviz::Panel)

namespace sdpo_ratf_ros_gui
{
    ratfPanel::ratfPanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::rviz_panel>())
    {   
        //ros::init(argc, argv, "sdpo_gui");
        ROS_INFO("Launching Gui Planning");
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        //ROS Subcribers

        connect(ui_->force_start, SIGNAL(clicked()), this, SLOT(forceStart()));
        connect(ui_->force_switch, SIGNAL(clicked()), this, SLOT(forceSwitch()));
    }

    void ratfPanel::forceStart()
    {
        ROS_INFO_STREAM("Force start pressed.");
        //this->button_1_pub_.publish(this->msg_);
    }

    void ratfPanel::forceSwitch()
    {
        ROS_INFO_STREAM("Force switch pressed.");
        //this->button_2_pub_.publish(this->msg_);
    }
} // namespace rviz_panel
