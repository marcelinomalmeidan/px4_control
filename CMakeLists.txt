cmake_minimum_required(VERSION 2.8.3)
project(px4_control)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  message_generation
  tf
  cmake_modules
)

add_message_files(DIRECTORY msg FILES
   PVA.msg
 )

find_package(Eigen REQUIRED)

#Add service files
add_service_files( FILES
  updatePx4param.srv
)

generate_messages( DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
	DEPENDS Eigen
#  INCLUDE_DIRS include
#  LIBRARIES px4_control
#  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp std_msgs visualization_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(px4_control_node 
	           src/px4_control.cpp 
	           src/HelperFunctions/helper.cpp
	           src/HelperFunctions/QuatRotEuler.cpp
             src/Callbacks/callbacks.cpp
             src/Threads/FSMTask.cpp
             src/Threads/joyTask.cpp
             src/Threads/commPub.cpp
             src/JoyDrivers/joyDrivers.cpp
             src/PosControl/PosControl.cpp
             src/pevents/pevents.cpp
             src/Services/services.cpp
             src/structs.cpp)
target_link_libraries(px4_control_node ${catkin_LIBRARIES})

#This makes sure that messages and services are compiled before the rest
add_dependencies(px4_control_node px4_control_generate_messages_cpp)
add_dependencies(px4_control_node ${catkin_EXPORTED_TARGETS})

