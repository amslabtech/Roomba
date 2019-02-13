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
 * 				  :       Takato  Saito (AMSL)
 *
 * CopyRight    :       2011, Autonomous Mobile Systems Laboratory, Meiji Univ.
 *
 * Revision     :       2012/05/17
 *
 */
//-----------------------------------------------------------------------------
#include <ros/ros.h>
//#include <joy/Joy.h>
#include <sensor_msgs/Joy.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <boost/thread.hpp>
boost::mutex cntl_mutex_;
boost::mutex std_mutex;

#include <iostream>
#include <math.h>
#include <time.h>
#include <std_msgs/String.h>
using namespace std;

inline int toInt(string s){int v; istringstream sin(s);sin>>v;return v;}
template<class T> inline string toString(T x) {ostringstream sout;sout<<x;return sout.str();}

//joy::Joy joy_in;
sensor_msgs::Joy joy_in;
ros::Publisher pub_state;

bool flag = false;
clock_t s,e;
int l[10] = {-1};
int c = 0;

void stdCallback(const std_msgs::StringConstPtr& msg)
{
	boost::mutex::scoped_lock(std_mutex);
	//cout<<"--"<<endl;
	int a = toInt(msg->data);
	int count = 0;
	int run = 0;
	
	for(int i=0;i<10;i++) if(l[i] == a) count++;
	if(count == 0 && a <= 10 && c <=10){
		l[c] = a;
		c++;
	}
	else run = 1;
	
	if(run ==0) flag = true;
}

void joy_callback(const sensor_msgs::JoyConstPtr& msg){
	boost::mutex::scoped_lock(cntl_mutex_);
	//joy_in=*msg;
	roomba_500driver_meiji::RoombaCtrl ctrl;
	
	ctrl.mode=roomba_500driver_meiji::RoombaCtrl::DRIVE;
	/*
	if(msg->buttons[0]){
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::SAFE;
		cout<<"SAFE mode"<<endl;
	}
	
	if(msg->buttons[1]){
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::SPOT;
		cout<<"SPOT mode"<<endl;
	}
	
	if(msg->buttons[2]){
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::CLEAN;
		cout<<"CLEAN mode"<<endl;
	}
	
	if(msg->buttons[3]){
		//ctrl.mode=roomba_500driver_meiji::RoombaCtrl::MAX;
		//cout<<"MAX mode"<<endl;
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::DOCK;
		cout<<"DOCK mode"<<endl;
	}
	
	if(msg->buttons[4]){
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::MOTORS;
		cout<<"MOTORS mode"<<endl;
	}
	if(msg->buttons[6]){
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::MOTORS_OFF;
		cout<<"MOTORS OFF mode"<<endl;
	}
	if(msg->buttons[5]){
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::FORCE_SEEK_DOCK;
		cout<<"FORCE_SEEK_DOCK mode"<<endl;
	}
	if(msg->buttons[7]){
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::FULL;
		cout<<"FULL mode"<<endl;
	}
	
	if(msg->buttons[9]){	//for red controller
	//if(msg->buttons[10]){
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::WAKEUP;
		cout<<"WAKEUP"<<endl;
	}

	if(msg->buttons[8]){	//for red
	//if(msg->buttons[11]){
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::POWER;
		cout<<"POWER OFF"<<endl;
	}*/
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
	if(flag){
		s = clock();
		cout<<"STOP"<<endl;
		clock_t sec = 0.0;
		while(sec < 10.0){
			ctrl.cntl.linear.x=0.0;
			ctrl.cntl.angular.z=0.0;
			ctrl.mode=roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
			pub_state.publish(ctrl);
			e = clock();
			sec = (double)(e-s)/CLOCKS_PER_SEC;
		}
		flag = false;
		
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::SONG;
		pub_state.publish(ctrl);

		ctrl.cntl.linear.x=0.0;
		ctrl.cntl.angular.z=0.0;
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
		cout<<ctrl.cntl.linear.x<<"   "<<ctrl.cntl.angular.z<<endl;
	}
	else if(msg->axes[1] || msg->axes[0]){ // 
		ctrl.cntl.linear.x=0.005*roomba_500driver_meiji::RoombaCtrl::DEFAULT_VELOCITY*msg->axes[1];
		ctrl.cntl.angular.z=(float)msg->axes[0];
	
		ctrl.mode=roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
		//cout<<ctrl.velocity<<"   "<<ctrl.radius<<endl;
		cout<<ctrl.cntl.linear.x<<"   "<<ctrl.cntl.angular.z<<endl;
		//pub_state.publish(ctrl);
	}	
	
	pub_state.publish(ctrl);
}



int main(int argc, char** argv) {
	
    ros::init(argc, argv, "joystick_drive");
    ros::NodeHandle n;

	pub_state=n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 100);
	//ros::Publisher pub_odom= n.advertise<geometry_msgs::TransformStamped >("/global_frame", 100);
	ros::Subscriber cntl_sub = n.subscribe("/joy", 100, joy_callback);
	ros::Subscriber std_sub_ = n.subscribe("qrcode_data", 1, stdCallback);
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

