#ifndef _H_JOY_THREAD_
#define _H_JOY_THREAD_

#include "ros/ros.h"
#include "../JoyDrivers/joyDrivers.h"
#include "../structs.h"
#include "../globals.h"

void *joyTaskTimer(void *threadID);

void *joyTask(void *threadID);

#endif