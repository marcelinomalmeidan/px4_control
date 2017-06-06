#ifndef _H_POSCONTROL__
#define _H_POSCONTROL_

#include <math.h>
#include "nav_msgs/Odometry.h"
#include "../structs.h"

//Sets initial errors to zero
void initializePID(PID_3DOF &PID);

//Sets integral error to zero
void resetIntegralErrorPID(PID_3DOF &PID);

//Update Kp, Ki and Kd in the PID
void updateControlParamPID(PID_3DOF &PID, 
	                       Eigen::Vector3d K_p, 
	                       Eigen::Vector3d K_i, 
	                       Eigen::Vector3d K_d, 
	                       Eigen::Vector3d maxInteg);

//Update all errors
void updateErrorPID(PID_3DOF &PID, 
	                Eigen::Vector3d feedForward, 
	                Eigen::Vector3d e_prop, 
	                Eigen::Vector3d e_deriv, 
	                float dt);

//Calculate output of PID
Eigen::Vector3d outputPID(PID_3DOF PID);

//Initialize parameters for position control
void initializePosControlParam(PosControlParam &Param,
	                           double mass, double gz,
	                           double thrustRatio);

//Load parameters from ROS parameter server
void readROSparameterServer(PID_3DOF &PID, PosControlParam &Param);

//Quadcopter position control
void PosController(nav_msgs::Odometry Odom,
	               PVA_structure PVA_ref,
	               PosControlParam Param,
	               double dt,
	               PID_3DOF &PosPID,
	               geometry_msgs::PoseStamped &PoseRef,
	               std_msgs::Float64 &refThrust);

#endif