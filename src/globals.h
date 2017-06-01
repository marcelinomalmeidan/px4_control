#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include "std_msgs/Float64.h"
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"

#include "structs.h"

// All global variables within the code
extern PVA_structure PVA_ref;
extern mavros_msgs::State PX4state;
extern nav_msgs::Odometry odom;
extern joyStruct joy;
extern char joyDriver[];
extern mutexStruct mutexes;
extern joyEventList joyEvents;
extern syncEventList syncEvents;
extern StateMachine FSM;
extern int threadCount;
extern PID_3DOF PosPID;
extern PosControlParam ControlParam;
