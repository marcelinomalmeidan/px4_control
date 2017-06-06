#include "ros/ros.h"
#include "../JoyDrivers/joyDrivers.h"
#include "../globals.h"

//Get Px4 state
void stateCallback(const mavros_msgs::State::ConstPtr &msg);

//Get odometry for the quadcopter
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

//Get joystick values based on chosen driver
void joyCallback(const sensor_msgs::Joy msg);

//Get PVA references from a ROS topic
void PVACallback(const px4_control::PVA::ConstPtr &msg);