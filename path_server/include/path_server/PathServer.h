#pragma once

// used for:
// -publishing saved path
// -saving a path to file.
// -recording and saving path

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h> 

class PathServer {
public:
    using UpdateFunc = bool (PathServer::*)();
    enum class Command {LOAD, SAVE, RECORD};
    PathServer(ros::NodeHandle* pn);

    bool update();
private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool saveUpdate();
    bool loadUpdate();
    bool recordUpdate();

    ros::Publisher pathPub;
    ros::Subscriber pathSub;
    ros::Subscriber odomSub;
    nav_msgs::Path path;

    double resolution;
    std::string filename;
    UpdateFunc fp;
};