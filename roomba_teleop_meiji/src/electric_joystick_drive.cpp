// Copyright 2023 amsl
//---------------------------< /-/ AMSL /-/ >------------------------------
/**
 * file         :       joystick_drive.cpp
 *
 *
 * Environment  :       g++
 * Latest Update:       2011/05/24
 *
 * Designer(s)  :       t.saitoh (AMSL)
 * Author(s)    :       m.mitsuhashi (AMSL)
 *                                :       Takato  Saito (AMSL)
 *
 * CopyRight    :       2011, Autonomous Mobile Systems Laboratory, Meiji Univ.
 *
 * Revision     :       2012/05/17
 *
 */
//-----------------------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

#include <math.h>
#include <time.h>
#include <iostream>
using namespace std;

class JoystickDrive : public rclcpp::Node
{
public:
  JoystickDrive()
  : Node("joystick_drive")
    , l_{-1}
  {
    ctrl_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>(
      "/roomba/control", 10
    );
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoystickDrive::joy_callback, this, std::placeholders::_1)
    );
  }

private:
  void string_callback(const std::shared_ptr<std_msgs::msg::String> msg)
  {
    const int a = toInt(msg->data);
    int count = 0;
    int run = 0;

    for (int i = 0; i < 10; i++) {
      if (l_[i] == a) {
        count++;
      }
    }
    if (count == 0 && a <= 10 && c_ <= 10) {
      l_[c_] = a;
      c_++;
    } else {
      run = 1;
    }

    if (run == 0) {
      flag_ = true;
    }
  }

  void joy_callback(const std::shared_ptr<sensor_msgs::msg::Joy> msg)
  {
    roomba_500driver_meiji::msg::RoombaCtrl ctrl;
    ctrl.mode = roomba_500driver_meiji::msg::RoombaCtrl::DRIVE;
    if (flag_) {
      RCLCPP_INFO_STREAM(this->get_logger(), "STOP");
      s_ = clock_.now();
      while (s_ + rclcpp::Duration(10, 0) > clock_.now()) {
        ctrl.cntl.linear.x = 0.0;
        ctrl.cntl.angular.z = 0.0;
        ctrl.mode = roomba_500driver_meiji::msg::RoombaCtrl::DRIVE_DIRECT;
        ctrl_pub_->publish(ctrl);
      }
      flag_ = false;

      ctrl.mode = roomba_500driver_meiji::msg::RoombaCtrl::SONG;
      ctrl_pub_->publish(ctrl);

      ctrl.cntl.linear.x = 0.0;
      ctrl.cntl.angular.z = 0.0;
      ctrl.mode = roomba_500driver_meiji::msg::RoombaCtrl::DRIVE;
      RCLCPP_INFO_STREAM(
        this->get_logger(), ctrl.cntl.linear.x << " " << ctrl.cntl.angular.z
      );
    } else if (msg->axes[1] || msg->axes[0]) {
      ctrl.cntl.linear.x =
        0.005 * roomba_500driver_meiji::msg::RoombaCtrl::DEFAULT_VELOCITY *
        msg->axes[1];
      ctrl.cntl.angular.z = msg->axes[0];
      RCLCPP_INFO_STREAM(
        this->get_logger(), ctrl.cntl.linear.x << " " << ctrl.cntl.angular.z
      );
    }
    ctrl_pub_->publish(ctrl);
  }

  int toInt(std::string & s)
  {
    int v;
    std::istringstream sin(s);
    sin >> v;
    return v;
  }

  template<class T>
  std::string toString(T x)
  {
    std::ostringstream sout;
    sout << x;
    return sout.str();
  }

  sensor_msgs::msg::Joy joy_in_;
  rclcpp::Publisher<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr
    ctrl_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  bool flag_;
  rclcpp::Time s_;
  rclcpp::Clock clock_;
  int l_[10];
  int c_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickDrive>());
  rclcpp::shutdown();

  return 0;
}
