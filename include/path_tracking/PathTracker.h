#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <string>

class PathTracker {
public:
    PathTracker(std::string throttleTopic, std::string brakeTopic,
                std::string steeringTopic, std::string odomTopic);
    PathTracker(const PathTracker&) = delete;
    PathTracker& operator=(const PathTracker&) = delete;
protected:
    ros::Publisher throttlePub;
    ros::Publisher brakePub;
    ros::Publisher steeringPub;
    ros::Subscriber odomSub;

    virtual void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg) = 0;
};