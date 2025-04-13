// Copyright 2025 amsl

#ifndef ROOMBA_500DRIVER_MEIJI__TWIST_TO_ROOMBACTRL_CONVERTER_HPP_
#define ROOMBA_500DRIVER_MEIJI__TWIST_TO_ROOMBACTRL_CONVERTER_HPP_

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roomba_500driver_meiji/msg/roomba_ctrl.hpp>

namespace roomba_500driver_meiji
{
class TwistToRoombactrlConverter : public rclcpp::Node
{
public:
  TwistToRoombactrlConverter(void);

  void cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

private:
  rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr
    ctrl_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace roomba_500driver_meiji

#endif  // ROOMBA_500DRIVER_MEIJI__TWIST_TO_ROOMBACTRL_CONVERTER_HPP_
