#include "path_tracking/PathTracker.h"
#include "path_tracking/utilities.h"

#include <std_msgs/Float64.h>

#define AUDIBOT_STEERING_RATIO  17.3

PathTracker::PathTracker(std::string throttleTopic, std::string brakeTopic,
                        std::string steeringTopic, std::string odomTopic,
                        std::string pathTopic)
{
    ros::NodeHandle nh;
    constexpr int queueSize = 10;
    throttlePub = nh.advertise<std_msgs::Float64>(throttleTopic, queueSize);
    brakePub = nh.advertise<std_msgs::Float64>(brakeTopic, queueSize);
    steeringPub = nh.advertise<std_msgs::Float64>(steeringTopic, queueSize);
    odomSub = nh.subscribe(odomTopic, queueSize, &PathTracker::callbackOdom, this);
    pathSub = nh.subscribe(pathTopic, queueSize, &PathTracker::callbackPath, this);
}

void PathTracker::callbackPath(const nav_msgs::Path::ConstPtr& msg){
    // path_msg = msg;
}

Stanley::Stanley(ros::NodeHandle* pn) : 
    PathTracker{pn->param<std::string>("throttleTopic", "throttle_cmd"),
                pn->param<std::string>("brakeTopic", "brake_cmd"),
                pn->param<std::string>("steeringTopic", "steering_cmd"),
                pn->param<std::string>("odomTopic", "odom"),
                pn->param<std::string>("pathTopic", "path")}
{
    path_msg = new nav_msgs::Path{};
    path_msg->poses.push_back({});
    path_msg->poses.push_back({});
    path_msg->poses.push_back({});
    path_msg->poses[0].pose.position.x = 0;
    path_msg->poses[0].pose.position.y = 0;
    path_msg->poses[0].pose.position.z = 0;
    path_msg->poses[1].pose.position.x = 50;
    path_msg->poses[1].pose.position.y = 0;
    path_msg->poses[1].pose.position.z = 0;
    path_msg->poses[2].pose.position.x = 50;
    path_msg->poses[2].pose.position.y = 100;
    path_msg->poses[2].pose.position.z = 0;

    ROS_INFO("Stanley class initialized");
}

void Stanley::callbackOdom(const nav_msgs::Odometry::ConstPtr& msg){
    if (path_msg->poses.size() < 2){
        return;
    }
    constexpr double Kp = -0.1;
    unsigned int closest = utilities::closetPoint(msg->pose.pose.position, path_msg->poses);
    if (closest >= path_msg->poses.size()-1){
        closest = path_msg->poses.size() - 2;
    }
    double headingError = yawError(msg->pose.pose.orientation, closest);
    double cte = utilities::crossTrackError(msg->pose.pose.position,
                                            path_msg->poses[closest].pose.position,
                                            path_msg->poses[closest+1].pose.position);
    double vel = utilities::velocity(*msg);
    double steeringAngle = headingError + atan(Kp*cte / (1 + vel));
    ROS_INFO_STREAM("Steering angle: " << steeringAngle*180/M_PI << "[deg], closest: " << closest << ", cte: " << cte);
    std_msgs::Float64 temp;
    temp.data = AUDIBOT_STEERING_RATIO * steeringAngle;
    steeringPub.publish(temp);
    temp.data = 0.2;
    throttlePub.publish(temp);
}

double Stanley::yawError(const geometry_msgs::Quaternion& quat, unsigned int closest){
    double carYaw = utilities::getYaw(quat);
    double pathYaw = utilities::getPathYaw(closest, path_msg->poses);
    ROS_INFO_STREAM("car: " << 180/M_PI*carYaw << ", path: " << 180/M_PI*pathYaw);
    return pathYaw - carYaw;
}