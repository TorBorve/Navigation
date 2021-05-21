#include "path_tracking/PathTracker.h"

#include <std_msgs/Float64.h>

PathTracker::PathTracker(std::string throttleTopic, std::string brakeTopic,
                        std::string steeringTopic, std::string odomTopic)
{
    ros::NodeHandle nh;
    constexpr int queueSize = 10;
    throttlePub = nh.advertise<std_msgs::Float64>(throttleTopic, queueSize);
    brakePub = nh.advertise<std_msgs::Float64>(brakeTopic, queueSize);
    steeringPub = nh.advertise<std_msgs::Float64>(steeringTopic, queueSize);
    odomSub = nh.subscribe(odomTopic, queueSize, &PathTracker::callbackOdom, this);
}