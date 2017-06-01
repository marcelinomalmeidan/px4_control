#ifndef _H_FSM_THREAD_
#define _H_FSM_THREAD_


#include "../globals.h"
#include "ros/ros.h"

//State Machine task
void *FSMTask(void *threadID);

#endif