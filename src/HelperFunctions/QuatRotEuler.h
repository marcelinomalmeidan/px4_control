#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include <math.h>
#include <Eigen/Dense>

geometry_msgs::Quaternion zeroQuaternion();

geometry_msgs::Quaternion setQuat(float qx, float qy, float qz, float qw);

geometry_msgs::Quaternion quatInv(geometry_msgs::Quaternion quat);

geometry_msgs::Quaternion quatProd(geometry_msgs::Quaternion q1,
	                               geometry_msgs::Quaternion q2);

geometry_msgs::Quaternion normalizeQuat(geometry_msgs::Quaternion q);

geometry_msgs::Vector3 quat2rpy(geometry_msgs::Quaternion quat);

geometry_msgs::Quaternion rpy2quat(geometry_msgs::Vector3 rpy);

double getHeadingFromQuat(geometry_msgs::Quaternion quat);

Eigen::Matrix3d quat2rot(geometry_msgs::Quaternion quat);

geometry_msgs::Quaternion rot2quat(Eigen::Matrix3d R);

Eigen::Matrix3d rotx(double theta);

Eigen::Matrix3d roty(double theta);

Eigen::Matrix3d rotz(double theta);