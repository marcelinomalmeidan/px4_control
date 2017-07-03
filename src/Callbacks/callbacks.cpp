#include "callbacks.h"

//Get current state of the vehicle
void stateCallback(const mavros_msgs::State::ConstPtr &msg){
	pthread_mutex_lock(&mutexes.PX4state);
		PX4state = *msg;
	pthread_mutex_unlock(&mutexes.PX4state);
	// std::cout << state.header.seq << std::endl;
}

//Get data from odometry topic
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
	pthread_mutex_lock(&mutexes.odom);
		odom = *msg;
	pthread_mutex_unlock(&mutexes.odom);
}

void tfCallback(const nav_msgs::Odometry::ConstPtr &msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
		                            msg->pose.pose.position.y,
		                            msg->pose.pose.position.z) );
  	transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,
  		                                 msg->pose.pose.orientation.y,
  		                                 msg->pose.pose.orientation.z,
  		                                 msg->pose.pose.orientation.w));

 	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fcu", "quad"));
}

//Get data from joysticks using appropriate driver
void joyCallback(const sensor_msgs::Joy msg){

	//Load appropriate drivers based on controller
	if(joyDriver.compare("joyXboxOne") == 0){
		pthread_mutex_lock(&mutexes.joy);
	    	joy = driverXboxOne(msg);
	    pthread_mutex_unlock(&mutexes.joy);
	}
	else if(joyDriver.compare("joyXbox360") == 0){ 
		pthread_mutex_lock(&mutexes.joy);
	    	joy = driverXbox360(msg);
	    pthread_mutex_unlock(&mutexes.joy);
	}
	else if(joyDriver.compare("joyXbox360Wired") == 0){
		pthread_mutex_lock(&mutexes.joy);
	    	joy = driverXbox360Wired(msg);
	    pthread_mutex_unlock(&mutexes.joy);
	}
	else{
	    ROS_ERROR("Joystick driver not found!\n");
	}
	    
	// printJoyValues(joy);
}

void PVACallback(const px4_control::PVA::ConstPtr &msg){
	pthread_mutex_lock(&mutexes.PVA_ros);
    	PVA_Ros = *msg;
    pthread_mutex_unlock(&mutexes.PVA_ros);
}