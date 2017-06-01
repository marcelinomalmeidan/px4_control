#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include <math.h>
#include <Eigen/Dense>
  

geometry_msgs::Vector3 SetVector3(float x, float y, float z);

geometry_msgs::Vector3 AddVector3(geometry_msgs::Vector3 Vec1,
                                  geometry_msgs::Vector3 Vec2);

geometry_msgs::Vector3 SubtractVector3(geometry_msgs::Vector3 Vec1,
                                       geometry_msgs::Vector3 Vec2);

geometry_msgs::Vector3 ZeroVector3();

float normVector3(geometry_msgs::Vector3 Vec3);

geometry_msgs::Point Vec3_2_Point(geometry_msgs::Vector3 Vec3);

geometry_msgs::Point SetPoint(float x, float y, float z);

geometry_msgs::Point AddPoint(geometry_msgs::Point Pt1,
                              geometry_msgs::Point Pt2);

geometry_msgs::Point SubtractPoint(geometry_msgs::Point Pt1,
                                   geometry_msgs::Point Pt22);

geometry_msgs::Point ZeroPoint();

float normPoint(geometry_msgs::Point Pt);

Eigen::Matrix3d skew(float x, float y, float z);

Eigen::Vector3d normalizeVector3d(Eigen::Vector3d V);

nav_msgs::Path getEmptyPathMsg();

/* Minimum function */
float min(double x, double y);

/* Maximum function */
float max(double x, double y);

// Saturate a value between minimum boundary and maximum boundary
float saturate(double in, double min_val, double max_val);

