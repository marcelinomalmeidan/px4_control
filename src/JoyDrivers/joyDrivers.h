#ifndef _H_JOY_DRIVERS_
#define _H_JOY_DRIVERS_

#include "sensor_msgs/Joy.h"
#include "../structs.h"
#include "../HelperFunctions/helper.h"

joyStruct driverXboxOne(sensor_msgs::Joy msg);

joyStruct driverXbox360(sensor_msgs::Joy msg);

joyStruct driverXbox360Wired(sensor_msgs::Joy msg);

PVA_structure filterJoy(PVA_structure PVA_refPrev,
                        geometry_msgs::Vector3 Vel_ref,
                        double dt);

void printJoyValues(joyStruct joy);

#endif