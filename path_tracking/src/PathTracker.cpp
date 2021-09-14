#include "path_tracking/PathTracker.h"
#include "path_tracking/utilities.h"

#include <std_msgs/Float64.h>

#define AUDIBOT_STEERING_RATIO  17.3

#ifdef CMAKE_CXX_FLAGS_DEBUG
#define LOG_INFO(info) \
    ROS_INFO_STREAM(info)
#else
#define LOG_INFO(info) ((void)0)
#endif


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

    while(ros::ok() && !ros::topic::waitForMessage<nav_msgs::Path>(pathTopic, ros::Duration{10.0})){
        ROS_WARN_STREAM("Waiting for first path message");
    }
    while(ros::ok() && !ros::topic::waitForMessage<nav_msgs::Odometry>(odomTopic, ros::Duration{10.0})){
        ROS_WARN_STREAM("Waiting for first odom message");
    }
    ROS_INFO_STREAM("PathTracker class initialized");
}

void PathTracker::callbackPath(const nav_msgs::Path::ConstPtr& msg){
    path_msg = *msg;
}

Stanley::Stanley(ros::NodeHandle* pn) : 
    PathTracker{pn->param<std::string>("throttleTopic", "throttle_cmd"),
                pn->param<std::string>("brakeTopic", "brake_cmd"),
                pn->param<std::string>("steeringTopic", "steering_cmd"),
                pn->param<std::string>("odomTopic", "odom"),
                pn->param<std::string>("pathTopic", "path")}
{
    ROS_INFO("Stanley class initialized");
}

void Stanley::callbackOdom(const nav_msgs::Odometry::ConstPtr& msg){
    if (path_msg.poses.size() < 2){
        return;
    }
    constexpr double Kp = -0.7;
    unsigned int closest = utilities::closetPoint(msg->pose.pose.position, path_msg.poses);
    if (closest >= path_msg.poses.size()-1){
        closest = path_msg.poses.size() - 2;
    }
    double headingError = yawError(msg->pose.pose.orientation, closest);
    double cte = utilities::crossTrackError(msg->pose.pose.position,
                                            path_msg.poses[closest].pose.position,
                                            path_msg.poses[closest+1].pose.position);
    double vel = utilities::velocity(*msg);
    double steeringAngle = headingError + atan(Kp*cte / (1 + vel));
    LOG_INFO("Steering angle: " << steeringAngle*180/M_PI << "[deg], closest: " << closest << ", cte: " << cte << ", vel: " << vel);
    std_msgs::Float64 temp;
    temp.data = AUDIBOT_STEERING_RATIO * steeringAngle;
    steeringPub.publish(temp);
    temp.data = 0.20;
    throttlePub.publish(temp);
}

double Stanley::yawError(const geometry_msgs::Quaternion& quat, unsigned int closest){
    double carYaw = utilities::getYaw(quat);
    double pathYaw = utilities::getPathYaw(closest, path_msg.poses);
    LOG_INFO("car: " << 180/M_PI*carYaw << ", path: " << 180/M_PI*pathYaw);
    return utilities::validAngle(pathYaw - carYaw);
}