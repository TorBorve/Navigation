#pragma once

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>


namespace utilities{

double getYaw(const geometry_msgs::Quaternion& quat);

double getPathYaw(unsigned int ind, const std::vector<geometry_msgs::PoseStamped>& path);

double getYaw(const geometry_msgs::Point& start, const geometry_msgs::Point& end);

unsigned int closetPoint(const geometry_msgs::Point& p, const std::vector<geometry_msgs::PoseStamped>& path);

double distSqrd(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);

double crossTrackError(const geometry_msgs::Point& vehicle, const geometry_msgs::Point& start, const geometry_msgs::Point& end);

double velocity(const nav_msgs::Odometry& odom);

double velocity(const geometry_msgs::Vector3& vec);
}