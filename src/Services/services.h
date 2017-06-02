
#include "px4_control/updatePx4param.h"
#include "../PosControl/PosControl.h"
#include "ros/ros.h"
#include "../structs.h"
#include "../globals.h"

//Update values in controller such as PID terms
bool updatePosControlParam(px4_control::updatePx4param::Request &req,
	                       px4_control::updatePx4param::Response &res);

//Update mass, thrustRatio and gravity
bool updateSystemParam(px4_control::updatePx4param::Request &req,
	                   px4_control::updatePx4param::Response &res);