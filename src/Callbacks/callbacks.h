#include "ros/ros.h"
#include "../JoyDrivers/joyDrivers.h"
#include "../globals.h"


void stateCallback(const mavros_msgs::State::ConstPtr &msg);

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

void joyCallback(const sensor_msgs::Joy msg);