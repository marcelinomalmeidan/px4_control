clc
clear all;
close all;

%You can set the parameters below if ROS is not running on the current machine
% setenv('ROS_MASTER_URI','http://192.168.1.20:11311')
% setenv('ROS_IP','192.168.1.20')

rosshutdown;
rosinit;

%Vehicle parameters
mass = 0.407;
gz = 9.81;
thrustRatio = 2.3;

%Controller parameters
Kpx = 1.7;
Kpy = 1.7;
Kpz = 1.2;
Kvx = 0.8;
Kvy = 0.8;
Kvz = 0.7;
Kix = 0;
Kiy = 0;
Kiz = 0;
maxInteg_x = 0;
maxInteg_y = 0;
maxInteg_z = 0;

%Create services
Param_srv = rossvcclient('/px4_control_node/updateQuadParam');
PID_srv = rossvcclient('/px4_control_node/updatePosControlParam');

%Populate structures
ParamVec = [mass gz thrustRatio];
Param = rosmessage('px4_control/updatePx4paramRequest');
Param.Data = ParamVec;

PID_Vec = [Kpx Kpy Kpz Kvx Kvy Kvz Kix Kiy Kiz maxInteg_x maxInteg_y maxInteg_z];
PID = rosmessage('px4_control/updatePx4paramRequest');
PID.Data = PID_Vec;

%Send new values
response = call(Param_srv,Param,'Timeout',2.0)
response = call(PID_srv,PID,'Timeout',2.0)