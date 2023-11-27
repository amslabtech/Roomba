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
 *
 * CopyRight    :       2011, Autonomous Mobile Systems Laboratory, Meiji Univ.
 *
 * Revision     :       2011/05/24
 *
 */
//-----------------------------------------------------------------------------
#include <ros/ros.h>

#include <joy/Joy.h>
// #include <sensor_msgs/Joy.h>

#include <roomba_500driver_meiji/RoombaCtrl.h>

#include <boost/thread.hpp>
boost::mutex cntl_mutex_;

#include <iostream>
#include <math.h>

joy::Joy joy_in;
ros::Publisher pub_state;

void joy_callback(const joy::JoyConstPtr& msg)
{
  boost::mutex::scoped_lock(cntl_mutex_);
  // joy_in=*msg;
  roomba_500driver_meiji::RoombaCtrl ctrl;

  ctrl.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE;

  if (msg->buttons[0])
  {
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::SAFE;
    std::cout << "SAFE mode" << std::endl;
  }

  if (msg->buttons[1])
  {
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::SPOT;
    std::cout << "SPOT mode" << std::endl;
  }

  if (msg->buttons[2])
  {
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::CLEAN;
    std::cout << "CLEAN mode" << std::endl;
  }

  if (msg->buttons[3])
  {
    // ctrl.mode=roomba_500driver_meiji::RoombaCtrl::MAX;
    // std::cout<<"MAX mode"<<std::endl;
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::DOCK;
    std::cout << "DOCK mode" << std::endl;
  }

  if (msg->buttons[4])
  {
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::MOTORS;
    std::cout << "MOTORS mode" << std::endl;
  }
  if (msg->buttons[6])
  {
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::MOTORS_OFF;
    std::cout << "MOTORS OFF mode" << std::endl;
  }
  if (msg->buttons[5])
  {
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::FORCE_SEEK_DOCK;
    std::cout << "FORCE_SEEK_DOCK mode" << std::endl;
  }
  if (msg->buttons[7])
  {
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::FULL;
    std::cout << "FULL mode" << std::endl;
  }

  if (msg->buttons[9])
  {  // for red controller
     // if(msg->buttons[10]){
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::WAKEUP;
    std::cout << "WAKEUP" << std::endl;
  }

  if (msg->buttons[8])
  {  // for red
     // if(msg->buttons[11]){
    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::POWER;
    std::cout << "POWER OFF" << std::endl;
  }
  /*
	ctrl.velocity=1.0*roomba_msgs::RoombaCtrl::DEFAULT_VELOCITY*msg->axes[1]+
					roomba_msgs::RoombaCtrl::DEFAULT_VELOCITY*msg->axes[5];

	ctrl.radius=((float)ctrl.velocity/msg->axes[2])+0*roomba_msgs::RoombaCtrl::STRAIGHT_RADIUS;

	if(fabs(ctrl.radius)>roomba_msgs::RoombaCtrl::STRAIGHT_RADIUS){
		ctrl.radius=roomba_msgs::RoombaCtrl::STRAIGHT_RADIUS;
	}

	if(msg->axes[4]){ //
		ctrl.velocity=roomba_msgs::RoombaCtrl::DEFAULT_VELOCITY;
		ctrl.radius=msg->axes[4];
	}
	*/
  if (msg->axes[1] || msg->axes[2])
  {  //
    ctrl.cntl.linear.x = 0.002 * roomba_500driver_meiji::RoombaCtrl::DEFAULT_VELOCITY * msg->axes[1];
    ctrl.cntl.angular.z = static_cast<float>(msg->axes[2]);

    ctrl.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
    // std::cout<<ctrl.velocity<<"   "<<ctrl.radius<<std::endl;
    std::cout << ctrl.cntl.linear.x << "   " << ctrl.cntl.angular.z << std::endl;
  }

  pub_state.publish(ctrl);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_drive");
  ros::NodeHandle n;

  pub_state = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 100);
  // ros::Publisher pub_odom= n.advertise<geometry_msgs::TransformStamped >("/global_frame", 100);
  ros::Subscriber cntl_sub = n.subscribe("/joy", 100, joy_callback);
  /*
	ros::Rate loop_rate(20);
	while (ros::ok()) {

		ros::spinOnce();
        loop_rate.sleep();
    }
	*/

  ros::spin();
  return 0;
}
