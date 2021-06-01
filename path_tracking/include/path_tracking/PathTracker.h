#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <string>

/// @brief Abstract class for path tracking algorithms. 
/// Used to publish commands for the vehicle and subscribe to path and odometry topic.
class PathTracker {
public:
    /// @brief Constructor for PathTracker.
    /// @param[in] throttleTopic topic name for throttle command sent to vehicle.
    /// @param[in] brakeTopic topic name for brake command sent to vehicle.
    /// @param[in] steeringTopic topic name for steering angle sent to vehicle.
    /// @param[in] odomTopic topic name where odometry is provided.
    /// @param[in] pathTopic topic name where desired path is provided.
    PathTracker(std::string throttleTopic, std::string brakeTopic,
                std::string steeringTopic, std::string odomTopic,
                std::string pathTopic);

    /// @brief copy constructor deleted.
    PathTracker(const PathTracker&) = delete;

    /// @brief copy assignment operator deleted.
    PathTracker& operator=(const PathTracker&) = delete;
protected:
    /// @brief publisher to throttle topic.
    ros::Publisher throttlePub;

    /// @brief publisher to brake topic.
    ros::Publisher brakePub;

    /// @brief publisher to steering topic.
    ros::Publisher steeringPub;

    /// @brief subscriber to odometry topic.
    ros::Subscriber odomSub;

    /// @brief subscirber to path topic.
    ros::Subscriber pathSub;

    /// @brief path message recived from path topic. Vehicle follows this path.
    nav_msgs::Path path_msg;

    /// @brief callback function for odometry. Has to be implemented by the derived class.
    /// @param[in] msg Odometry message recived from odometry topic.
    virtual void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg) = 0;

    /// @brief callback function for path. Updates path_msg member variable.
    /// @param[in] msg Path message recived from path topic.
    virtual void callbackPath(const nav_msgs::Path::ConstPtr& msg);
};

/// @brief Uses stanley steering for path tracking. Inherits for PathTracker class.
class Stanley : public PathTracker {
public:
    /// @brief Constructor for Stanley class.
    /// @param[in] pn pointer to private NodeHandle.
    Stanley(ros::NodeHandle* pn);
private:
    /// @brief callback function for Odometry. Overrides callback funtion in
    /// base class. 
    /// @param[in] msg Odometry message recived from odometry topic.
    void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg) override;

    /// @brief calculates the yaw error between path tanget and vehicle.
    /// @param[in] quat vehicle orientation as Quaternion.
    /// @param[in] closest index of closest point to the vehicle on the path.
    double yawError(const geometry_msgs::Quaternion& quat, unsigned int closest);
};