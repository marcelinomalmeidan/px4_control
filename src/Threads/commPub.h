#ifndef _H_COMMPUB_THREAD_
#define _H_COMMPUB_THREAD_

#include "ros/ros.h"
#include "../JoyDrivers/joyDrivers.h"
#include "../PosControl/PosControl.h"
#include "../structs.h"
#include "../globals.h"

void *commPubTimer(void *threadID);

void *commPubTask(void *threadID);

#endif