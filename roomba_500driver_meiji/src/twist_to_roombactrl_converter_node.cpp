// Copyright 2025 amsl

#include <rclcpp/rclcpp.hpp>
#include <roomba_500driver_meiji/twist_to_roombactrl_converter.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<roomba_500driver_meiji::TwistToRoombactrlConverter>()
  );
  rclcpp::shutdown();
  return 0;
}
