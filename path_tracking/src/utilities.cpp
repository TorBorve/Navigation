#include "path_tracking/utilities.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace utilities {

double getYaw(const geometry_msgs::Quaternion& quat){
    tf2::Quaternion q{quat.x, quat.y, quat.z, quat.w};
    tf2::Matrix3x3 mat{q};
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    return yaw;
}

double getPathYaw(unsigned int ind, const std::vector<geometry_msgs::PoseStamped>& path){
    return getYaw(path[ind].pose.position, path[ind+1].pose.position);
}

double getYaw(const geometry_msgs::Point& start, const geometry_msgs::Point& end){
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    return atan2(dy, dx);
}

unsigned int closetPoint(const geometry_msgs::Point& p, const std::vector<geometry_msgs::PoseStamped>& path){
    double minDist = distSqrd(p, path[0].pose.position);
    unsigned int minInd = 0;
    for (unsigned int i = 1; i < path.size(); i++){
        double dist = distSqrd(p, path[i].pose.position);
        if (dist < minDist){
            minDist = dist;
            minInd = i;
        }
    }
    return minInd;
}

double distSqrd(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
    return (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) + (p2.z-p1.z)*(p2.z-p1.z); /// z or not???
}

double crossTrackError(const geometry_msgs::Point& vehicle, const geometry_msgs::Point& start, const geometry_msgs::Point& end){
    double pathYaw = getYaw(start, end);
    return cos(pathYaw)*(vehicle.y-start.y)-sin(pathYaw)*(vehicle.x-start.x);
}

double velocity(const nav_msgs::Odometry& odom){
    return length(odom.twist.twist.linear);
}

double length(const geometry_msgs::Vector3& vec){
    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

double validAngle(double angle){
    if (angle > M_PI){
        angle-= 2*M_PI;
    } else if (angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

}