#include "PosControl.h"


void PosController(nav_msgs::Odometry Odom,
	               PVA_structure PVA_ref,
	               PosControlParam Param,
	               double dt,
	               PID_3DOF &PosPID,
	               geometry_msgs::PoseStamped &PoseRef,
	               std_msgs::Float64 &refThrust){
	double m = Param.mass;
	double gz = Param.gz;
	double maxThrust = Param.thrustRatio*m*gz;

	double yawRef;							//Desired yaw
	Eigen::Vector3d e_Pos, e_Vel; 			//error in position and velocity
	Eigen::Vector3d PosRef, VelRef, AccRef;	//Desired PVA
	Eigen::Vector3d Pos, Vel;				//Current PV
	Eigen::Vector3d feedForward;			//Feedforward vector
	Eigen::Vector3d Fdes;					//Desired force in body frame
	Eigen::Vector3d z_w, z_b;				//z direction in world and body frames 
	Eigen::Matrix3d Rbw;					//Rotation from body to world
	Eigen::Matrix3d Rdes;					//Desired rotation
	Eigen::Vector3d z_bdes, x_cdes, y_bdes, x_bdes;

	//Desired states
	PosRef << PVA_ref.Pos.pose.position.x,
	          PVA_ref.Pos.pose.position.y,
	          PVA_ref.Pos.pose.position.z;
	VelRef << PVA_ref.Vel.twist.linear.x,
	          PVA_ref.Vel.twist.linear.y,
	          PVA_ref.Vel.twist.linear.z;
	AccRef << PVA_ref.Acc.accel.linear.x,
	          PVA_ref.Acc.accel.linear.y,
	          PVA_ref.Acc.accel.linear.z;
	yawRef = getHeadingFromQuat(Odom.pose.pose.orientation);

	//Current states
	Pos << Odom.pose.pose.position.x,
	       Odom.pose.pose.position.y,
	       Odom.pose.pose.position.z;
	Vel << Odom.twist.twist.linear.x,
	       Odom.twist.twist.linear.y,
	       Odom.twist.twist.linear.z;

	//Rotation matrix and z frame vectors
	Rbw = quat2rot(Odom.pose.pose.orientation);
	z_w << 0, 0, 1;
	z_b = Rbw*z_w;

	//Calculate errors (Obs: Odom might be in body or inertial frame)
	e_Pos = PosRef - Pos;
	std::string frameId = Odom.header.frame_id;
  	std::string childFrameId = Odom.child_frame_id;
	if(frameId.compare(childFrameId) == 0){
		e_Vel = VelRef - Vel;
	}
	else{
		e_Vel = VelRef - Rbw*Vel;
	}

	//Translational controller
	feedForward = m*gz*z_w + m*AccRef;
	updateErrorPID(PosPID, feedForward, e_Pos, e_Vel, dt);
	Fdes = outputPID(PosPID);

	//Desired thrust in body frame
	refThrust.data = min(max(Fdes.dot(z_b), 0.0),maxThrust)/maxThrust;

	//Find desired attitude from desired force and yaw angle
	z_bdes = normalizeVector3d(Fdes);
	x_cdes << 1, 0, 0;
	y_bdes = normalizeVector3d(z_bdes.cross(x_cdes));
	x_bdes = y_bdes.cross(z_bdes);
	Rdes << x_bdes, y_bdes, z_bdes;

	PoseRef.pose.position = PVA_ref.Pos.pose.position;
	PoseRef.pose.orientation = rot2quat(Rdes);

}