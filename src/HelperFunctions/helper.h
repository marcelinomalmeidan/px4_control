#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include <math.h>
#include <Eigen/Dense>
  
//Function to help setting data into a ROS Vector3
geometry_msgs::Vector3 SetVector3(float x, float y, float z);

//Function to help adding two ROS Vector3
geometry_msgs::Vector3 AddVector3(geometry_msgs::Vector3 Vec1,
                                  geometry_msgs::Vector3 Vec2);

//Function to help subtracting two ROS Vector3
geometry_msgs::Vector3 SubtractVector3(geometry_msgs::Vector3 Vec1,
                                       geometry_msgs::Vector3 Vec2);

//Function to set a ROS Vector3 to zero
geometry_msgs::Vector3 ZeroVector3();

//Function to calculate the 2-norm of a ROS Vector3
float normVector3(geometry_msgs::Vector3 Vec3);

//Function to copy a ROS Vector3 into a ROS Point structure
geometry_msgs::Point Vec3_2_Point(geometry_msgs::Vector3 Vec3);

//Function to set data into a ROS Point
geometry_msgs::Point SetPoint(float x, float y, float z);

//Function to add two ROS Points
geometry_msgs::Point AddPoint(geometry_msgs::Point Pt1,
                              geometry_msgs::Point Pt2);

//Function to subtract two ROS Points
geometry_msgs::Point SubtractPoint(geometry_msgs::Point Pt1,
                                   geometry_msgs::Point Pt22);

//Function to set a ROS Point to zero
geometry_msgs::Point ZeroPoint();

//Function to calculate the 2-norm of a ROS Point
float normPoint(geometry_msgs::Point Pt);

//Function to print the values of a ROS Point (debug purposes)
void printPoint(geometry_msgs::Point Pt);

//Function to copy a ROS Point into a ROS Vector3 structure
geometry_msgs::Vector3 Point_2_Vec3(geometry_msgs::Point Pt);

//Function to generate a skew-symmetric matrix from a vector (based on kinematics)
Eigen::Matrix3d skew(float x, float y, float z);

//Function to normalize a vector to be unit-norm
Eigen::Vector3d normalizeVector3d(Eigen::Vector3d V);

/* Minimum function */
float min(double x, double y);

/* Maximum function */
float max(double x, double y);

// Saturate a value between minimum boundary and maximum boundary
float saturate(double in, double min_val, double max_val);

//Convert degree to radians
double deg2rad(double degVal);

//Convert radians to degrees
double rad2deg(double radVal);