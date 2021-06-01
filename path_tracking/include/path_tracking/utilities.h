#pragma once

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>


namespace utilities{

/// @brief calculates yaw from quaternion.
/// @param[in] quat orientation as quaternion.
/// @return yaw of quaternion.
double getYaw(const geometry_msgs::Quaternion& quat);

/// @brief calculates yaw of path given path and first index.
/// @param[in] ind index for first of the two points makeing the line.
/// @param[in] path path message that contains the points.
/// @return yaw of line between points corresponding to ind and ind+1
double getPathYaw(unsigned int ind, const std::vector<geometry_msgs::PoseStamped>& path);

/// @brief calculates yaw of line between two points.
/// @param[in] start first of the two points.
/// @param[in] end last of the two points.
/// @return yaw of line from start to end.
double getYaw(const geometry_msgs::Point& start, const geometry_msgs::Point& end);

/// @brief Finds the closest point on a path to a fixed point.
/// @param[in] p point to find closest point to.
/// @param[in] path path.
/// @return index of point on path that is cloest to p.
unsigned int closetPoint(const geometry_msgs::Point& p, const std::vector<geometry_msgs::PoseStamped>& path);

/// @brief calculates the distance between two points squared.
/// @param[in] p1 first point.
/// @param[in] p2 second point.
/// @return distance between p1 and p1 squared.
double distSqrd(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);

/// @brief calculates the cross track error between point and line give by two points.
/// @param[in] vehicle point of vehicle.
/// @param[in] start first of two points making a line.
/// @param[in] end last of two point making a line.
/// @return distance from vehicle to line. postive if to the left. Negative if on the right.
double crossTrackError(const geometry_msgs::Point& vehicle, const geometry_msgs::Point& start, const geometry_msgs::Point& end);

/// @brief calculates speed.
/// @param[in] odom odom message that contains velocity vector.
/// @return velocity given by odom message.
double velocity(const nav_msgs::Odometry& odom);

/// @brief calculates length of vector.
/// @param[in] vec 3D vector.
/// @return length of 3D vector.
double length(const geometry_msgs::Vector3& vec);

/// @brief makes sure angle is in interval [-PI, PI]
double validAngle(double angle);

}