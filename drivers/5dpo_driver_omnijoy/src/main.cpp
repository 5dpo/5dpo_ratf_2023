#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

namespace sdpo_driver_omnijoy {

class Omnijoy {
 public:
  Omnijoy();

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linearx_,lineary_, angular_, deadman_axis_, turbo_axis_ , turbo_up_axis_, turbo_down_axis_;
  double l_scale_, a_scale_,l_turbo_maxscale_,l_turbo_scale_,a_turbo_maxscale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  geometry_msgs::Twist last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  bool turbo_pressed_;
  bool turbo_up_pressed_;
  bool turbo_down_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;
};

Omnijoy::Omnijoy():
  ph_("~"),
  linearx_(1),
  lineary_(2),
  angular_(0),
  deadman_axis_(4),
  turbo_axis_(5),
  turbo_up_axis_(6),
  turbo_down_axis_(7),
  l_scale_(0.1),
  a_scale_(0.2),
  l_turbo_scale_(0.2),
  l_turbo_maxscale_(0.4),
  a_turbo_maxscale_(0.4)
{
  ph_.param("axis_linear_x", linearx_, linearx_);
  ph_.param("axis_linear_y", lineary_, lineary_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("axis_turbo", turbo_axis_, turbo_axis_);
  ph_.param("axis_turbo_up", turbo_up_axis_, turbo_up_axis_);
  ph_.param("axis_turbo_down", turbo_down_axis_, turbo_down_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);
  ph_.param("turbo_scale_linear", l_turbo_scale_, l_turbo_scale_);
  ph_.param("turbo_max_scale_linear", l_turbo_maxscale_, l_turbo_maxscale_);
  ph_.param("turbo_scale_angular", a_turbo_maxscale_, a_turbo_maxscale_);


  deadman_pressed_ = false;
  turbo_pressed_ = false;
  turbo_up_pressed_ = false;
  turbo_down_pressed_ = false;
  zero_twist_published_ = false;

  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Omnijoy::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&Omnijoy::publish, this));
}

void Omnijoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;
  vel.angular.z = joy->axes[angular_];
  vel.linear.x = joy->axes[linearx_];
  vel.linear.y = joy->axes[lineary_];
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_];
  turbo_pressed_ = joy->buttons[turbo_axis_];
  turbo_up_pressed_ = joy->buttons[turbo_up_axis_];
  turbo_down_pressed_ = joy->buttons[turbo_down_axis_];

}

void Omnijoy::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if(turbo_up_pressed_ && !turbo_down_pressed_){
      l_turbo_scale_=l_turbo_scale_+0.05;
      if(l_turbo_scale_>l_turbo_maxscale_){
          l_turbo_scale_=l_turbo_maxscale_;
      }
  }
  if(!turbo_up_pressed_ && turbo_down_pressed_){
      l_turbo_scale_=l_turbo_scale_-0.05;
      if(l_turbo_scale_<l_scale_ ){
          l_turbo_scale_=l_scale_;
      }
  }

  if (deadman_pressed_ && !turbo_pressed_)
  {
    last_published_.linear.y=(last_published_.linear.y*l_scale_);
    last_published_.linear.x=last_published_.linear.x*l_scale_;
    last_published_.angular.z=last_published_.angular.z*a_scale_;
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }else if (deadman_pressed_ && turbo_pressed_)
  {
    last_published_.linear.y=(last_published_.linear.y*l_turbo_scale_);
    last_published_.linear.x=last_published_.linear.x*l_turbo_scale_;
    last_published_.angular.z=last_published_.angular.z*a_turbo_maxscale_;
    vel_pub_.publish(last_published_);
    zero_twist_published_=false;
  }
  else if(!deadman_pressed_ && !zero_twist_published_)
  {
    vel_pub_.publish(*new geometry_msgs::Twist());
    zero_twist_published_=true;
  }
}

} // namespace sdpo_driver_omnijoy

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sdpo_driver_omnijoy");
  sdpo_driver_omnijoy::Omnijoy omni_joy;

  ros::spin();
}
