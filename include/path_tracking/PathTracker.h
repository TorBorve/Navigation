#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <string>

class PathTracker {
public:
    PathTracker(std::string throttleTopic, std::string brakeTopic,
                std::string steeringTopic, std::string odomTopic,
                std::string pathTopic);
    PathTracker(const PathTracker&) = delete;
    PathTracker& operator=(const PathTracker&) = delete;
protected:
    ros::Publisher throttlePub;
    ros::Publisher brakePub;
    ros::Publisher steeringPub;
    ros::Subscriber odomSub;
    ros::Subscriber pathSub;

    nav_msgs::Path* path_msg;

    virtual void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg) = 0;
    virtual void callbackPath(const nav_msgs::Path::ConstPtr& msg);
};

class Stanley : public PathTracker {
public:
    Stanley(ros::NodeHandle* pn);
    ~Stanley(){
        delete path_msg;
    } 
private:
    void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg) override;
    double yawError(const geometry_msgs::Quaternion& quat, unsigned int closest);
};