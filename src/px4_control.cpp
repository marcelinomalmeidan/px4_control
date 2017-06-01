#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"

#include "structs.h"
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"
#include "Callbacks/callbacks.h"
#include "Threads/FSMTask.h"
#include "Threads/joyTask.h"
#include "Threads/commPub.h"

#include <pthread.h>
#include "boost/bind.hpp"

#include <sstream>
#include <iostream>

// Global variables
PVA_structure PVA_ref;
mavros_msgs::State PX4state;
nav_msgs::Odometry odom;
joyStruct joy;
joyEventList joyEvents;
syncEventList syncEvents;
mutexStruct mutexes;
StateMachine FSM;
int threadCount = 0;
PID_3DOF PosPID;
PosControlParam ControlParam;


//Initialization settings
char odomTopic[] = "/mavros/local_position/odom";
char paramFile[] = "gazebo.csv"; //dragonfly gazebo
char joyDriver[] = "joyXboxOne"; //joyXbox360 joyXbox360Wired joyXboxOne


int main(int argc, char **argv)
{
  //Initialize ROS
  printf("something\n");
  ros::init(argc, argv, "controlPkg");
  ros::NodeHandle n;  

  //Initialize some variables
  initializePVA(PVA_ref);
  initializeJoy(joy);
  initializeEvents(joyEvents, syncEvents);
  initializeMutexes(mutexes);
  initializeStateMachine(FSM);

  printCurrentState(FSM);

  readROSparameterServer(PosPID, ControlParam);
  

  //Publishers -----------------------------------------------
  ros::Publisher attPub    = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",10);
  ros::Publisher thrustPub = n.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle",10);
  ros::Publisher posPub    = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);

  //Subscribers ----------------------------------------------
  ros::Subscriber stateSub = n.subscribe("mavros/state", 10, stateCallback);
  ros::Subscriber odomSub = n.subscribe(odomTopic, 10, odomCallback);
  ros::Subscriber joySub = n.subscribe("joy", 10, joyCallback);

  //Threads --------------------------------------------------
  pthread_t h_FSMThread;      //Finite state machine
  pthread_t h_joyThreadTimer; //Timer for joystick thread
  pthread_t h_joyThread;      //Joystick thread
  pthread_t h_commPubTimer;   //Timer for command publisher
  pthread_t h_commPubThread;  //Command publisher thread
  int ReturnCode;

  //Start  finite state machine
  if (ReturnCode = pthread_create(&h_FSMThread, NULL, FSMTask, NULL)){
    printf("Start FSM failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
        threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }

  //Start joystick timer thread
  if (ReturnCode = pthread_create(&h_joyThreadTimer, NULL, joyTaskTimer, NULL)){
    printf("Start joystick timer thread failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
        threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }


  //Start joystick thread
  if (ReturnCode = pthread_create(&h_joyThread, NULL, joyTask, NULL)){
    printf("Start joystick thread failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
        threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }

    //Start command publisher timer thread
  if (ReturnCode = pthread_create(&h_commPubTimer, NULL, commPubTimer, NULL)){
    printf("Start command publisher timer thread failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
        threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }


  //Start command publisher thread
  if (ReturnCode = pthread_create(&h_commPubThread, NULL, commPubTask, NULL)){
    printf("Start command publisher thread failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
        threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }


  //Start loop ----------------------------------------------------
  ros::Rate loop_rate(200);

  int localThreadCount;
  while (ros::ok())
  {

    //Check if all threads were terminated
    pthread_mutex_lock(&mutexes.threadCount);
        localThreadCount = threadCount;
    pthread_mutex_unlock(&mutexes.threadCount);
    if(localThreadCount == 0){
      break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  //Terminate program ---------------------------------------------
  destroyEvents(joyEvents, syncEvents);
  destroyMutexes(mutexes);

  ROS_INFO("Process Ended with Success!");


  return 0;

}