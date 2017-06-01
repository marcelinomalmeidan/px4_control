#ifndef _H_POSCONTROL__
#define _H_POSCONTROL_

#include <math.h>
#include "nav_msgs/Odometry.h"
#include "../structs.h"

void PosController(nav_msgs::Odometry Odom,
	               PVA_structure PVA_ref,
	               PosControlParam Param,
	               double dt,
	               PID_3DOF &PosPID,
	               geometry_msgs::PoseStamped &PoseRef,
	               std_msgs::Float64 &refThrust);

#endif