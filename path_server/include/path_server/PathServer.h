#pragma once

// used for:
// -publishing saved path
// -saving a path to file.
// -recording and saving path

#include <nav_msgs/Path.h>
#include <ros/ros.h> 

// void fun(){
//     PathServer ps{"jan", "nei", PathServer::Command::LOAD, 2, "Wjn"};
// }

class PathServer {
public:
    enum class Command {LOAD, SAVE, RECORD};
    PathServer();
private:
    ros::Publisher pathPub;
    ros::Subscriber pathSub;
    nav_msgs::Path path;
};