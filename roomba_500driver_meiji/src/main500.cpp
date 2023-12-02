// Copyright 2023 amsl

#include <chrono>
#include <cmath>
#include <iostream>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roomba_500driver_meiji/msg/roomba500_state.hpp"
#include "roomba_500driver_meiji/msg/roomba_ctrl.hpp"
#include "roomba_500driver_meiji/roomba500sci.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class RoombaDriver : public rclcpp::Node
{
public:
  RoombaDriver()
  : Node("roomba_driver")
    , clock_(RCL_SYSTEM_TIME)
    , pre_enc_r_(0)
    , pre_enc_l_(0)
  {
    pose_.x = 0.0;
    pose_.y = 0.0;
    pose_.theta = 0.0;

    roomba_state_pub_ =
      this->create_publisher<roomba_500driver_meiji::msg::Roomba500State>("/roomba/states", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/roomba/odometry", 10);
    roomba_ctrl_sub_ = this->create_subscription<roomba_500driver_meiji::msg::RoombaCtrl>(
      "/roomba/control",
      10,
      std::bind(&RoombaDriver::handle_roomba_control, this, std::placeholders::_1)
    );
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->declare_parameter("port", "/dev/ttyUSB0");
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("odom_frame", "odom");

    roomba_ = std::make_unique<roombaSci>(B115200, this->get_parameter("port").as_string().c_str());
    roomba_->wakeup();
    roomba_->startup();

    current_time_ = clock_.now();
    last_time_ = clock_.now();

    // Initialization of timer should be last.
    timer_ = this->create_wall_timer(100ms, std::bind(&RoombaDriver::timer_callback, this));
  }

  ~RoombaDriver()
  {
    if (roomba_) {
      roomba_->powerOff();
      roomba_->time_->sleep(1);
    }
  }

private:
  void handle_roomba_control(const std::shared_ptr<roomba_500driver_meiji::msg::RoombaCtrl> msg)
  {
    roombactrl_ = *msg;

    switch (msg->mode) {
      case roomba_500driver_meiji::msg::RoombaCtrl::SPOT:
        roomba_->spot();
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::SAFE:
        roomba_->safe();
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::CLEAN:
        roomba_->clean();
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::POWER:
        roomba_->powerOff();
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::WAKEUP:
        roomba_->wakeup();
        roomba_->startup();
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::FULL:
        roomba_->full();
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::MAX:
        roomba_->max();
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::DOCK:
        roomba_->dock();
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::MOTORS:
        roomba_->driveMotors(
          static_cast<roombaSci::MOTOR_BITS>(
            roombaSci::MB_MAIN_BRUSH | roombaSci::MB_SIDE_BRUSH | roombaSci::MB_VACUUM
        ));
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::MOTORS_OFF:
        roomba_->driveMotors((roombaSci::MOTOR_BITS)(0));
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::DRIVE_DIRECT:
        roomba_->driveDirect(msg->cntl.linear.x, msg->cntl.angular.z);
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::DRIVE_PWM:
        roomba_->drivePWM(msg->r_pwm, msg->l_pwm);
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::SONG:
        roomba_->safe();
        roomba_->song(1, 1);
        roomba_->playing(1);
        break;

      case roomba_500driver_meiji::msg::RoombaCtrl::DRIVE:
      default:
        roomba_->drive(msg->velocity, msg->radius);
    }
  }

  void timer_callback()
  {
    current_time_ = clock_.now();

    roomba_500driver_meiji::msg::Roomba500State sens;
    sens.header.stamp = current_time_;

    roomba_->getSensors(sens);

    int enc_r = roomba_->dEncoderRight();
    if (abs(enc_r) == 200) {
      enc_r = pre_enc_r_;
    }
    int enc_l = roomba_->dEncoderLeft();
    if (abs(enc_l) == 200) {
      enc_l = pre_enc_l_;
    }

    geometry_msgs::msg::Pose2D pre = pose_;

    float distance = (enc_r + enc_l) / 2270.0 * 0.5;
    float angle = (enc_r - enc_l) / 2270.0 / 0.235;
    sens.distance = 1000 * distance;
    sens.angle = angle * 180.0 / M_PI;

    roomba_state_pub_->publish(sens);

    calcOdometry(pose_, pre, distance, angle);

    pre_enc_r_ = roomba_->dEncoderRight();
    pre_enc_l_ = roomba_->dEncoderLeft();

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    // ROSのOdometryには，6DOFを利用するのでyaw角から生成したquaternionを用いる
    geometry_msgs::msg::Quaternion odom_quat;
    tf2::Quaternion q(tf2::Vector3(0, 0, 1), pose_.theta);
    tf2::convert(q, odom_quat);

    // first, we'll publish the transform over tf
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = this->get_parameter("odom_frame").as_string();
    odom_trans.child_frame_id = this->get_parameter("robot_frame").as_string();

    odom_trans.transform.translation.x = pose_.x;
    odom_trans.transform.translation.y = pose_.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    tf_broadcaster_->sendTransform(odom_trans);  // 2013.10.09

    // next, we'll publish the odometry message over ROS
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = this->get_parameter("odom_frame").as_string();

    // set the position
    odom.pose.pose.position.x = pose_.x;
    odom.pose.pose.position.y = pose_.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = this->get_parameter("robot_frame").as_string();
    odom.twist.twist.linear.x = roombactrl_.cntl.linear.x;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = roombactrl_.cntl.angular.z;

    odom_pub_->publish(odom);  // 2013.10.09

    last_time_ = current_time_;

    RCLCPP_INFO(
      get_logger(),
      "dt:%f\t444444444l: %5d\tr:%5d\tdl %4d\tdr %4d\tx:%f\ty:%f\ttheta:%f",
      last_time_.seconds() - current_time_.seconds(),
      sens.encoder_counts.left,
      sens.encoder_counts.right,
      roomba_->dEncoderLeft(),
      roomba_->dEncoderRight(),
      pose_.x,
      pose_.y,
      pose_.theta / M_PI * 180.0
    );
  }

  double piToPI(double rad)
  {
    double ret = rad;

    if (rad > M_PI) {
      ret = rad - 2.0 * M_PI;
    }

    if (rad < -M_PI) {
      ret = rad + 2.0 * M_PI;
    }

    return ret;
  }

  void calcOdometry(
    geometry_msgs::msg::Pose2D & x, const geometry_msgs::msg::Pose2D & pre_x, float dist,
    float angle
  )
  {
    x.theta = pre_x.theta + angle;
    x.theta = piToPI(x.theta);
    x.x = pre_x.x + dist * cos(x.theta);
    x.y = pre_x.y + dist * sin(x.theta);
  }

  rclcpp::Publisher<roomba_500driver_meiji::msg::Roomba500State>::SharedPtr roomba_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr roomba_ctrl_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<roombaSci> roomba_;
  roomba_500driver_meiji::msg::RoombaCtrl roombactrl_;
  geometry_msgs::msg::Pose2D pose_;
  rclcpp::Clock clock_;
  rclcpp::Time current_time_;
  rclcpp::Time last_time_;
  int pre_enc_r_;
  int pre_enc_l_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoombaDriver>());
  rclcpp::shutdown();

  return 0;
}
