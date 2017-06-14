#ifndef _H_JOY_DRIVERS_
#define _H_JOY_DRIVERS_

#include "sensor_msgs/Joy.h"
#include "../structs.h"
#include "../HelperFunctions/helper.h"

//Driver for receiving data from a Xbox One controller
joyStruct driverXboxOne(sensor_msgs::Joy msg);

//Driver for receiving data from a Xbox 360 controller
joyStruct driverXbox360(sensor_msgs::Joy msg);

//Driver for receiving data from a wired Xbox 360 controller
joyStruct driverXbox360Wired(sensor_msgs::Joy msg);

//Function for low-pass filtering position references for quadcopter
PVA_structure filterJoy(PVA_structure PVA_refPrev,
                        geometry_msgs::Vector3 Vel_ref,
                        double dt, double TimeConstant);

//Function to print values received from joystick (debug purposes)
void printJoyValues(joyStruct joy);

#endif