// Copyright 2023 amsl

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"

namespace roomba_500driver_meiji
{
class TwistToRoombactrlConverter : public rclcpp::Node
{
public:
  TwistToRoombactrlConverter(void)
  : Node("twist_to_roombactrl_converter")
  {
    ctrl_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>(
      "roomba/control", 10
    );
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(
        &TwistToRoombactrlConverter::cmd_vel_callback, this,
        std::placeholders::_1
      )
    );
  }

  void cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
  {
    roomba_500driver_meiji::msg::RoombaCtrl control;
    control.mode = roomba_500driver_meiji::msg::RoombaCtrl::DRIVE_DIRECT;
    control.cntl.linear.x = msg->linear.x;
    control.cntl.angular.z = msg->angular.z;
    ctrl_pub_->publish(control);
  }

private:
  rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr
    ctrl_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace roomba_500driver_meiji

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<roomba_500driver_meiji::TwistToRoombactrlConverter>()
  );
  rclcpp::shutdown();
  return 0;
}
