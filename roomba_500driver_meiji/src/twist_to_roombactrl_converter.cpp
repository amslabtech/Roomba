// Copyright 2007-2023 amsl

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>

namespace roomba_500driver_meiji
{
class TwistToRoombactrlConverter
{
public:
  TwistToRoombactrlConverter(void)
  {
    ctrl_pub_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel",
                                 1,
                                 &TwistToRoombactrlConverter::cmd_vel_callback,
                                 this,
                                 ros::TransportHints().reliable().tcpNoDelay(true));
  }

  void cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg)
  {
    roomba_500driver_meiji::RoombaCtrl control;
    control.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    control.cntl.linear.x = msg->linear.x;
    control.cntl.angular.z = msg->angular.z;
    ctrl_pub_.publish(control);
  }

  void process(void)
  {
    ros::spin();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle local_nh_;
  ros::Publisher ctrl_pub_;
  ros::Subscriber cmd_vel_sub_;
};

}  // namespace roomba_500driver_meiji

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_to_roombactrl_converter");
  roomba_500driver_meiji::TwistToRoombactrlConverter trc;
  trc.process();
  return 0;
}
