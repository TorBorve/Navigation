#include "path_server/PathServer.h"
#include "path_server/fileHandling.h"

#include <string>


double distSqrd(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
    return (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) + (p2.z-p1.z)*(p2.z-p1.z);
}

double distSqrd(const nav_msgs::Path& path, const nav_msgs::Odometry& odom){
    return distSqrd(path.poses.back().pose.position, odom.pose.pose.position);
}

inline geometry_msgs::PoseStamped toPoseStamped(const nav_msgs::Odometry& odom){
    geometry_msgs::PoseStamped ps;
    ps.header = odom.header;
    ps.pose = odom.pose.pose;
    return ps;
}   

PathServer::PathServer(ros::NodeHandle* pn)
    : filePath{pn->param<std::string>("filePath", "myPath.txt")},
    resolution{pn->param<double>("resolution", 1.0)}
{
    std::string command = pn->param<std::string>("command", "not specified");
    std::string pathTopic = pn->param<std::string>("pathTopic", "path");
    std::string odomTopic = pn->param<std::string>("odomTopic", "odom");
    std::string odomFrame = pn->param<std::string>("odomFrame", "odom");

    ros::NodeHandle nh{};
    if (command == "load"){
        file::readPath(path, filePath);
        pathPub = nh.advertise<nav_msgs::Path>(pathTopic, 1);
        fp = &PathServer::loadUpdate;
        ROS_INFO("PathServer initialized with load command.");
    } else if (command == "save"){
        pathSub = nh.subscribe(pathTopic,1, &PathServer::pathCallback, this);
        fp = &PathServer::saveUpdate;
        ROS_INFO("PathServer initialized with save command.");
    } else if (command == "record"){
        fp = &PathServer::recordUpdate;
        path.header.frame_id = odomFrame;
        odomSub = nh.subscribe(odomTopic, 10, &PathServer::odomCallback, this);
        pathPub = nh.advertise<nav_msgs::Path>(pathTopic, 1);
        ROS_INFO("PathServer initialized with record command.");
    } else {
        ROS_ERROR_STREAM("invalid command: " << command << "\nValid commands are: load, save, record");
        throw std::runtime_error{"invalid command: " + command};
    }
}

PathServer::~PathServer(){
    if (fp = &PathServer::recordUpdate){
        file::savePath(path, filePath);
    }
}

void PathServer::pathCallback(const nav_msgs::Path::ConstPtr& msg){
    path = *msg;
}

void PathServer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    if (path.poses.size() == 0 || distSqrd(path, *msg) > resolution*resolution){
        path.poses.push_back(toPoseStamped(*msg));
    }
}

bool PathServer::update(){
    return (this->*fp)();
}

bool PathServer::saveUpdate(){
    // return true if path has been set
    // this means that the path has been saved
    return path.poses.size() > 0;
}

bool PathServer::loadUpdate(){
    path.header.stamp = ros::Time::now();
    pathPub.publish(path);
    return false;
}

bool PathServer::recordUpdate(){
    constexpr unsigned int saveInterval = 15;
    static auto lastSave = ros::Time::now();
    ros::Duration elapsed = ros::Time::now() - lastSave;
    if (elapsed.toSec() >= saveInterval){
        file::savePath(path, filePath);
        lastSave = ros::Time::now();
    }
    path.header.stamp = ros::Time::now();
    pathPub.publish(path);
    return false;
}