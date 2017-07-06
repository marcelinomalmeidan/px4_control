#include "helper.h"

geometry_msgs::Vector3 SetVector3(float x, float y, float z){
	geometry_msgs::Vector3 Vec;
	Vec.x = x;
	Vec.y = y;
	Vec.z = z;
	return Vec;
}

geometry_msgs::Vector3 AddVector3(geometry_msgs::Vector3 Vec1, 
	                              geometry_msgs::Vector3 Vec2){
	geometry_msgs::Vector3 Vec_out;
	Vec_out.x = Vec1.x + Vec2.x;
	Vec_out.y = Vec1.y + Vec2.y;
	Vec_out.z = Vec1.z + Vec2.z;
	return Vec_out;
}

geometry_msgs::Vector3 SubtractVector3(geometry_msgs::Vector3 Vec1,
                                       geometry_msgs::Vector3 Vec2){
	geometry_msgs::Vector3 Vec_out;
	Vec_out.x = Vec1.x - Vec2.x;
	Vec_out.y = Vec1.y - Vec2.y;
	Vec_out.z = Vec1.z - Vec2.z;
	return Vec_out;
}

geometry_msgs::Vector3 ZeroVector3(){
	geometry_msgs::Vector3 Vec;
	Vec.x = 0.0;
	Vec.y = 0.0;
	Vec.z = 0.0;
	return Vec;
}

float normVector3(geometry_msgs::Vector3 Vec3){
	double norm;
	norm = sqrt(pow(Vec3.x,2) + pow(Vec3.y,2) + pow(Vec3.z,2));
	return norm;
}

geometry_msgs::Point Vec3_2_Point(geometry_msgs::Vector3 Vec3){
	geometry_msgs::Point Pt;
	Pt.x = Vec3.x;
	Pt.y = Vec3.y;
	Pt.z = Vec3.z;
	return Pt;
}

geometry_msgs::Point SetPoint(float x, float y, float z){
	geometry_msgs::Point Pt;
	Pt.x = x;
	Pt.y = y;
	Pt.z = z;
	return Pt;
}

geometry_msgs::Point AddPoint(geometry_msgs::Point Pt1,
                              geometry_msgs::Point Pt2){
	geometry_msgs::Point Pt_out;
	Pt_out.x = Pt1.x + Pt2.x;
	Pt_out.y = Pt1.y + Pt2.y;
	Pt_out.z = Pt1.z + Pt2.z;
	return Pt_out;
}

geometry_msgs::Point SubtractPoint(geometry_msgs::Point Pt1,
                                   geometry_msgs::Point Pt2){
	geometry_msgs::Point Pt_out;
	Pt_out.x = Pt1.x - Pt2.x;
	Pt_out.y = Pt1.y - Pt2.y;
	Pt_out.z = Pt1.z - Pt2.z;
	return Pt_out;
}

geometry_msgs::Point ZeroPoint(){
	geometry_msgs::Point Pt;
	Pt.x = 0.0;
	Pt.y = 0.0;
	Pt.z = 0.0;
	return Pt;
}

float normPoint(geometry_msgs::Point Pt){
	double norm;
	norm = sqrt(pow(Pt.x,2) + pow(Pt.y,2) + pow(Pt.z,2));
	return norm;
}

void printPoint(geometry_msgs::Point Pt){
	ROS_INFO("Value %f %f %f", Pt.x, Pt.y, Pt.z);
}

geometry_msgs::Vector3 Point_2_Vec3(geometry_msgs::Point Pt){
	geometry_msgs::Vector3 Vec3;
	Vec3.x = Pt.x;
	Vec3.y = Pt.y;
	Vec3.z = Pt.z;
	return Vec3;
}

Eigen::Matrix3d skew(float x, float y, float z){
	Eigen::Matrix3d M;

	M <<  0, -z,  y,
	      z,  0, -x,
	     -y, x,  0;

	return M;
}

Eigen::Vector3d normalizeVector3d(Eigen::Vector3d V){
	double normV = V.norm();
	if (normV > 0){
		return V*(1.0 / normV);
	}
	else{
		return V;
	}
}

// nav_msgs::Path getEmptyPathMsg(){
// 	nav_msgs::Path PathMsg;
// 	PathMsg.header.seq = 1;
// 	PathMsg.header.stamp = ros::Time::now();
// 	PathMsg.header.frame_id = "fcu";

// 	return PathMsg;
// }

/* Minimum function */
float min(double x, double y)
{
	return (x < y) ? x : y;
}

/* Maximum function */
float max(double x, double y)
{
	return (x > y) ? x : y;
}

// Saturate a value between minimum boundary and maximum boundary
float saturate(double in, double min_val, double max_val){
	return min(max(in, min_val), max_val);
}

//Convert degree to radians
double deg2rad(double degVal){
	return degVal*M_PI/180.0;
}

//Convert radians to degrees
double rad2deg(double radVal){
	return radVal*180.0/M_PI;
}