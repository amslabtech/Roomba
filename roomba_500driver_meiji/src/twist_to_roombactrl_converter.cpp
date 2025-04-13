// Copyright 2023 amsl

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"
#include "roomba_500driver_meiji/twist_to_roombactrl_converter.hpp"

namespace roomba_500driver_meiji
{
TwistToRoombactrlConverter::TwistToRoombactrlConverter(void)
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

void TwistToRoombactrlConverter::cmd_vel_callback(
  const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  roomba_500driver_meiji::msg::RoombaCtrl control;
  control.mode = roomba_500driver_meiji::msg::RoombaCtrl::DRIVE_DIRECT;
  control.cntl.linear.x = msg->linear.x;
  control.cntl.angular.z = msg->angular.z;
  ctrl_pub_->publish(control);
}

}  // namespace roomba_500driver_meiji
