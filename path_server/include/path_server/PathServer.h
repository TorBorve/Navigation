#pragma once

// used for:
// -publishing saved path
// -saving a path to file.
// -recording and saving path

/// make saveupdate thread safe?

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h> 

/// @brief Class for path_server
class PathServer {
public:
    using UpdateFunc = bool (PathServer::*)();
    
    /// @brief constructor for PathServer
    /// @param[in] pn pointer to private nodeHandle.
    PathServer(ros::NodeHandle* pn);

    /// @brief destructor for PathServer.
    ~PathServer();

    /// @brief update function for calling fp. Should be used in spin once loop.
    /// @return true if finished else false.
    bool update();
private:
    /// @brief callback function for path topic.
    /// @param[in] msg path message from path topic.
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);

    /// @brief callback function for odometry topic.
    /// @param[in] msg odometry message from odometry topic.
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /// @brief update function for saving path.
    /// @return true if finished saving else false.
    bool saveUpdate();

    /// @brief update function for loading saved path.
    /// @return false.
    bool loadUpdate();

    /// @brief update function for recording path.
    /// @return false.
    bool recordUpdate();

    /// @brief publishes path message to path topic.
    ros::Publisher pathPub;

    /// @brief subscribes to path topic.
    ros::Subscriber pathSub;

    /// @brief subscribes to odometry topic. Used if recording path.
    ros::Subscriber odomSub;

    /// @brief path message.
    nav_msgs::Path path;

    /// @brief minimum distance before adding point to path when recording.
    double resolution;

    /// @brief file path for saving or reading path.
    std::string filePath;

    /// @brief function pointer to desired update function.
    UpdateFunc fp;
};