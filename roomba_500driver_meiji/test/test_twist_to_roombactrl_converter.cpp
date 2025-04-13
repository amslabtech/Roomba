// Copyright 2025 amsl

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <roomba_500driver_meiji/msg/roomba_ctrl.hpp>
#include <roomba_500driver_meiji/twist_to_roombactrl_converter.hpp>

class TwistToRoombactrlConverterTest : public ::testing::Test
{
public:
  void ctrl_callback(const roomba_500driver_meiji::msg::RoombaCtrl::SharedPtr msg)
  {
    received_control_ = msg;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<roomba_500driver_meiji::msg::RoombaCtrl>::SharedPtr ctrl_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  roomba_500driver_meiji::msg::RoombaCtrl::SharedPtr received_control_;

protected:
  void SetUp() override
  {
    // Initialize the node and publisher
    node_ = std::make_shared<rclcpp::Node>("test_twist_to_roombactrl_converter");
    ctrl_sub_ = node_->create_subscription<roomba_500driver_meiji::msg::RoombaCtrl>(
      "roomba/control", 10,
      std::bind(&TwistToRoombactrlConverterTest::ctrl_callback, this, std::placeholders::_1)
    );
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10
    );
  }

  void TearDown() override
  {
    received_control_.reset();
    ctrl_sub_.reset();
    node_.reset();
  }
};

TEST_F(TwistToRoombactrlConverterTest, cmd_vel)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  const auto converter_node =
    std::make_shared<roomba_500driver_meiji::TwistToRoombactrlConverter>();

  executor.add_node(converter_node);
  executor.add_node(node_);

  const auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
  twist_msg->linear.x = 1.0;
  twist_msg->angular.z = 0.5;

  cmd_vel_pub_->publish(*twist_msg);
  while (rclcpp::ok()) {
    if (received_control_) {
      break;
    }
    executor.spin_once();
  }
  EXPECT_EQ(received_control_->mode, roomba_500driver_meiji::msg::RoombaCtrl::DRIVE_DIRECT);
  EXPECT_EQ(received_control_->cntl.linear.x, twist_msg->linear.x);
  EXPECT_EQ(received_control_->cntl.angular.z, twist_msg->angular.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
