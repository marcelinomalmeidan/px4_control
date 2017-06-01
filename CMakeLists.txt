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
  cmake_modules
)

find_package(Eigen REQUIRED)

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
             src/structs.cpp)
target_link_libraries(px4_control_node ${catkin_LIBRARIES})
add_dependencies(px4_control_node beginner_tutorials_generate_messages_cpp)

