// Copyright 2007-2023 amsl
#include <ros/ros.h>

#include <roomba_500driver_meiji/roomba500sci.h>
#include <roomba_500driver_meiji/Roomba500State.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread.hpp>
boost::mutex cntl_mutex_;

#include <iostream>
#include <string>
#include <math.h>

// 2013.10.09...tfのodomをコメントアウト

roombaSci* roomba;
roomba_500driver_meiji::RoombaCtrl roombactrl;

void cntl_callback(const roomba_500driver_meiji::RoombaCtrlConstPtr& msg)
{
  roombactrl = *msg;
  boost::mutex::scoped_lock(cntl_mutex_);

  switch (msg->mode)
  {
    case roomba_500driver_meiji::RoombaCtrl::SPOT:
      roomba->spot();
      break;

    case roomba_500driver_meiji::RoombaCtrl::SAFE:
      roomba->safe();
      break;

    case roomba_500driver_meiji::RoombaCtrl::CLEAN:
      roomba->clean();
      break;

    case roomba_500driver_meiji::RoombaCtrl::POWER:
      roomba->powerOff();
      break;

    case roomba_500driver_meiji::RoombaCtrl::WAKEUP:
      roomba->wakeup();
      roomba->startup();
      break;

    case roomba_500driver_meiji::RoombaCtrl::FULL:
      roomba->full();
      break;

    case roomba_500driver_meiji::RoombaCtrl::MAX:
      roomba->max();
      break;

    case roomba_500driver_meiji::RoombaCtrl::DOCK:
      roomba->dock();
      break;

    case roomba_500driver_meiji::RoombaCtrl::MOTORS:
      roomba->driveMotors(
          (roombaSci::MOTOR_BITS)(roombaSci::MB_MAIN_BRUSH | roombaSci::MB_SIDE_BRUSH | roombaSci::MB_VACUUM));
      break;

    case roomba_500driver_meiji::RoombaCtrl::MOTORS_OFF:
      roomba->driveMotors((roombaSci::MOTOR_BITS)(0));
      break;

    case roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT:
      roomba->driveDirect(msg->cntl.linear.x, msg->cntl.angular.z);
      break;

    case roomba_500driver_meiji::RoombaCtrl::DRIVE_PWM:
      roomba->drivePWM(msg->r_pwm, msg->l_pwm);
      break;

    case roomba_500driver_meiji::RoombaCtrl::SONG:
      roomba->safe();
      roomba->song(1, 1);
      roomba->playing(1);
      break;

    case roomba_500driver_meiji::RoombaCtrl::DRIVE:
    default:
      roomba->drive(msg->velocity, msg->radius);
  }
}

void printSensors(const roomba_500driver_meiji::Roomba500State& sens)
{
  std::cout << "\n\n-------------------" << std::endl;

  // Roombaのセンサデータを出力しています
  // 必要ない場合はコメントアウトしてください

  std::cout << "bumps : "
            << static_cast<bool>(sens.bump.right)
            << "  "
            << static_cast<bool>(sens.bump.left) << std::endl;
  std::cout << "wheeldrops : "
            << static_cast<bool>(sens.wheeldrop.right)
            << "  "
            << static_cast<bool>(sens.wheeldrop.left)
            << "  "
            << static_cast<bool>(sens.wheeldrop.caster) << std::endl;
  std::cout << "wall : " << static_cast<bool>(sens.wall) << std::endl;
  std::cout << "cliff : "
            << static_cast<bool>(sens.cliff.left)
            << " "
            << static_cast<bool>(sens.cliff.right)
            << std::endl;
  std::cout << "virtual wall : " << static_cast<bool>(sens.virtual_wall) << std::endl;
  std::cout << "motor ovc : "
            << sens.motor_overcurrents.side_brush << " "
            << sens.motor_overcurrents.vacuum << " "
            << sens.motor_overcurrents.main_brush << " "
            << sens.motor_overcurrents.drive_right << " "
            << sens.motor_overcurrents.drive_left << " "
            << std::endl;

  std::cout << "dirt_detector : "
            << static_cast<int16_t>(sens.dirt_detector.left)
            << " "
            << static_cast<int16_t>(sens.dirt_detector.right)
            << std::endl;
  std::cout << "remote control command : " << static_cast<int16_t>(sens.remote_control_command) << std::endl;
  std::cout << "buttons : "
            << static_cast<bool>(sens.buttons.power)
            << " "
            << static_cast<bool>(sens.buttons.spot)
            << " "
            << static_cast<bool>(sens.buttons.clean)
            << " "
            << static_cast<bool>(sens.buttons.max)
            << std::endl;

  std::cout << "charging_state : " << static_cast<int16_t>(sens.charging_state) << std::endl;
  std::cout << "voltage : " << sens.voltage << std::endl;
  std::cout << "current : " << sens.current << std::endl;
  std::cout << "temperature : " << static_cast<int16_t>(sens.temperature) << std::endl;
  std::cout << "charge : " << sens.charge << std::endl;
  std::cout << "capacity : " << sens.capacity << std::endl;
}

double piToPI(double rad)
{
  double ret = rad;

  if (rad > M_PI)
  {
    ret = rad - 2.0 * M_PI;
  }

  if (rad < -M_PI)
  {
    ret = rad + 2.0 * M_PI;
  }

  return ret;
}

void calcOdometry(
    geometry_msgs::Pose2D& x,
    const geometry_msgs::Pose2D& pre_x,
    float dist,
    float angle)
{
  x.theta = pre_x.theta + angle;
  x.theta = piToPI(x.theta);
  x.x = pre_x.x + dist * cos(x.theta);
  x.y = pre_x.y + dist * sin(x.theta);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roomba_driver");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  ros::Subscriber cntl_sub = n.subscribe("/roomba/control", 100, cntl_callback);

  ros::Publisher pub_state = n.advertise<roomba_500driver_meiji::Roomba500State>("/roomba/states", 100);

  tf::TransformBroadcaster odom_broadcaster;

  ros::Publisher pub_odo = n.advertise<nav_msgs::Odometry>("/roomba/odometry", 100);

  std::string USB_PORT;
  private_nh.param("USB_PORT", USB_PORT, std::string("/dev/ttyUSB0"));
  roomba = new roombaSci(B115200, USB_PORT.c_str());
  roomba->wakeup();
  roomba->startup();

  std::string ROBOT_FRAME;
  private_nh.param("ROBOT_FRAME", ROBOT_FRAME, std::string("base_link"));
  std::string ODOM_FRAME;
  private_nh.param("ODOM_FRAME", ODOM_FRAME, std::string("odom"));

  ros::Rate loop_rate(10.0);  // should not set this rate more than 20Hz !

  geometry_msgs::Pose2D pose;
  pose.x = 0;
  pose.y = 0;
  pose.theta = 0;

  int pre_enc_r = 0, pre_enc_l = 0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  while (ros::ok())
  {
    current_time = ros::Time::now();

    roomba_500driver_meiji::Roomba500State sens;
    sens.header.stamp = ros::Time::now();

    {
      boost::mutex::scoped_lock(cntl_mutex_);
      roomba->getSensors(sens);
    }
    // printSensors(sens);

    int enc_r = roomba->dEncoderRight();
    if (abs(enc_r) == 200)
    {
      enc_r = pre_enc_r;
    }
    int enc_l = roomba->dEncoderLeft();
    if (abs(enc_l) == 200)
    {
      enc_l = pre_enc_l;
    }

    geometry_msgs::Pose2D pre = pose;

    const float distance = static_cast<float>(enc_r + enc_l) / 2270.0 * 0.5;
    const float angle = static_cast<float>(enc_r - enc_l) / 2270.0 / 0.235;
    sens.distance = static_cast<int16_t>(1000 * distance);
    sens.angle = static_cast<int16_t>(angle * 180.0 / M_PI);

    pub_state.publish(sens);

    calcOdometry(pose, pre, distance, angle);

    pre_enc_r = roomba->dEncoderRight();
    pre_enc_l = roomba->dEncoderLeft();

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    // ROSのOdometryには，6DOFを利用するのでyaw角から生成したquaternionを用いる
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.theta);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = ODOM_FRAME;  // 2013.10.09
    odom_trans.child_frame_id = ROBOT_FRAME;  // 2013.10.09

    odom_trans.transform.translation.x = pose.x;
    odom_trans.transform.translation.y = pose.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);  // 2013.10.09

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = ODOM_FRAME;  // 2013.10.09

    // set the position
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = ROBOT_FRAME;  // 2013.10.09
    odom.twist.twist.linear.x = roombactrl.cntl.linear.x;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = roombactrl.cntl.angular.z;

    pub_odo.publish(odom);  // 2013.10.09

    last_time = current_time;

    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO(
        "dt:%f\t444444444l: %5d\tr:%5d\tdl %4d\tdr %4d\tx:%f\ty:%f\ttheta:%f",
        last_time.toSec() - current_time.toSec(),
        sens.encoder_counts.left,
        sens.encoder_counts.right,
        roomba->dEncoderLeft(),
        roomba->dEncoderRight(),
        pose.x,
        pose.y,
        pose.theta / M_PI * 180.0);
  }

  roomba->powerOff();

  roomba->time_->sleep(1);

  delete roomba;

  return 0;
}
