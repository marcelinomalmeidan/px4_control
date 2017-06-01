#ifndef _H_JOY_THREAD_
#define _H_JOY_THREAD_

#include "ros/ros.h"
#include "../JoyDrivers/joyDrivers.h"
#include "../structs.h"
#include "../globals.h"

void loadJoyRefParam(double &RollMax, double &PitchMax, double &YawRateMax,
	                 double &maxThrust, double &xRate, double &yRate, double &zRate);

void *joyTaskTimer(void *threadID);

void *joyTask(void *threadID);

#endif