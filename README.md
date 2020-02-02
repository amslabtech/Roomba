# Roomba

## Install

### Environment

- Ubuntu 16.04 or Ubuntu 18.04
- ROS Kinetic or Melodic

### Compiling

```bash
sudo apt install ros-$ROS_DISTRO-joy
sudo apt install ros-$ROS_DISTRO-joystick-drivers
cd ~/catkin_ws/src/
git clone https://github.com/amslabtech/Roomba.git
catkin build
source ~/catkin_ws/devel/setup.bash
```

## Running this driver

### USB Permissions

Connect computer to 7-pin serial port

```bash
sudo chmod a+rw /dev/ttyUSB0
```

### Run

```bash
rosrun roomba_500driver_meiji main500
```

### Publishers

Topic | Description | Type
--- | --- | ---
`/roomba/states` | Robot satatus | [roomba_500driver_meiji/Roomba500State][roomba500State]
`/roomba/odometry` | Robot odometry according to wheel encoders | [nav_msgs::Odometry][odometry]

### Subscriber

Topic | Description | Type
--- | --- | ---
`/roomba/control` | Set mode and drives the robot's wheels according to a forward and angular velocity | [roomba_500driver_meiji/RoombaCtrl][roombactrl]

## Control Robot

You can move the robot around by sending [roomba_500driver_meiji/RoombaCtrl][roombactrl] messages to the topic `/roomba/control` :

```txt
mode                  Set 11
cntl.linear.x  (+)    Move forward
cntl.linear.x  (-)    Move backward
cntl.angular.z (+)    Rotate counter-clockwise
cntl.angular.z (-)    Rotate clockwise
```

### limits

`-1.0 <= cntl.linear.x <= 1.0` and `-1.0 <= cntl.angular.z <= 1.0`

### Teleoperation

Connect computer to joystick

```bash
rosrun joy joy_node
rosrun roomba_teleop_meiji electric_joystick_drive
```

[roomba500State]: https://github.com/amslabtech/Roomba/blob/master/roomba_500driver_meiji/msg/Roomba500State.msg
[odometry]: http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
[roombactrl]: https://github.com/amslabtech/Roomba/blob/master/roomba_500driver_meiji/msg/RoombaCtrl.msg