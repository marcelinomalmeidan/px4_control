#include "services.h"


bool updatePosControlParam(px4_control::updatePx4param::Request &req,
	                       px4_control::updatePx4param::Response &res){
	// std_msgs::Float64 Values;
	// req.push_back(Values.data);
	// Values = req.data;

	//Check for the right number of arguments
	if(req.data.size() != 12){
		ROS_INFO("Wrong number of parameters. You need to send parameters in the following order:\n kpx kpy kpz kvx kvy kvz kix kiy kyz maxInteg_x maxInteg_y maxInteg_z");
		return false;
	}

	//Set the parameters
	Eigen::Vector3d Kp, Ki, Kd, maxInteg;
	Kp << req.data[0], req.data[1], req.data[2];
	Kd << req.data[3], req.data[4], req.data[5];
	Ki << req.data[6], req.data[7], req.data[8];
	maxInteg << req.data[9], req.data[10], req.data[11];

	pthread_mutex_lock(&mutexes.PID_Pos);
		updateControlParamPID3(PosPID, Kp, Ki, Kd, maxInteg);
	pthread_mutex_unlock(&mutexes.PID_Pos);

  ROS_INFO("Kp: %f,\t%f,\t%f", Kp[0], Kp[1], Kp[2]);
  ROS_INFO("Kd: %f,\t%f,\t%f", Kd[0], Kd[1], Kd[2]);
  ROS_INFO("Ki: %f,\t%f,\t%f", Ki[0], Ki[1], Ki[2]);
  ROS_INFO("maxInteg: %f,\t%f,\t%f\n", maxInteg[0], maxInteg[1], maxInteg[2]);

  res.success = true;

	return true;
}

//Update mass, thrustRatio and gravity
bool updateSystemParam(px4_control::updatePx4param::Request &req,
	                   px4_control::updatePx4param::Response &res){

	//Check for the right number of arguments
	if(req.data.size() != 3){
		ROS_INFO("Wrong number of parameters. You need to send parameters in the following order:\n mass gz thrustRatio");
		return false;
	}
	
	//Set the parameters
	double mass, gz, thrustRatio;
	mass = req.data[0];
	gz = req.data[1];
	thrustRatio = req.data[2];

	pthread_mutex_lock(&mutexes.PID_Param);
		initializePosControlParam(ControlParam, mass, gz, thrustRatio);
	pthread_mutex_unlock(&mutexes.PID_Param);

	ROS_INFO("mass: %f\tgz: %f\tthrustRatio: %f", mass, gz, thrustRatio);

	res.success = true;

	return true;
}