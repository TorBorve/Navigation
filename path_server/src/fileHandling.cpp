// example of output file of path:

// header: 
//   seq: 69
//   stamp: 1622277861.888276134
//   frame_id: frame_id
// poses[]
//   poses[0]: 
//     header: 
//       seq: 69
//       stamp: 1622277861.888276134
//       frame_id: frame_id
//     pose: 
//       position: 
//         x: 1
//         y: 2
//         z: 3
//       orientation: 
//         x: 0
//         y: 0
//         z: 0
//         w: 1
//   poses[1]: 
//     header: 
//       seq: 69
//       stamp: 1622277861.888276134
//       frame_id: frame_id
//     pose: 
//       position: 
//         x: 1
//         y: 2
//         z: 3
//       orientation: 
//         x: 0
//         y: 0
//         z: 0
//         w: 1

#define BASEDIR "/home/torb/catkin_ws/src/navigation/path_server/src/"

#define LOAD_CHECK(lhs, rhs) \
    if(lhs != rhs){ROS_WARN_STREAM("load warning: " << lhs << " != " << rhs);}

#include "path_server/fileHandling.h"

#include <ros/ros.h>

#include <fstream>

namespace file{

void readHeader(std::ifstream& inFile, std_msgs::Header& header){
    std::string word;
    uint32_t num;
    double dub;
    inFile >> word; 
    LOAD_CHECK(word, "header:");
    inFile >> word;
    LOAD_CHECK(word, "seq:");
    inFile >> header.seq;
    inFile >> word;
    LOAD_CHECK(word, "stamp:");
    inFile >> num;
    header.stamp.sec = num;
    inFile.get();
    inFile >> num;
    header.stamp.nsec = num;
    // has to check if frame_id = "" this can cause problems later
    getline(inFile, word); // read rest of previous line
    getline(inFile, word); // read frame_id line
    std::stringstream ss{word};
    ss >> word;
    LOAD_CHECK(word, "frame_id:");
    if (!ss.eof()){
        ss >> header.frame_id;
    }
}

void readPosition(std::ifstream& inFile, geometry_msgs::Point& point){
    std::string word;
    inFile >> word;
    LOAD_CHECK(word, "position:");
    inFile >> word;
    LOAD_CHECK(word, "x:");
    inFile >> point.x;
    inFile >> word;
    LOAD_CHECK(word, "y:");
    inFile >> point.y;
    inFile >> word;
    LOAD_CHECK(word, "z:");
    inFile >> point.z;
}

void readOrientation(std::ifstream& inFile, geometry_msgs::Quaternion& quat){
    std::string word;
    inFile >> word;
    LOAD_CHECK(word, "orientation:");
    inFile >> word;
    LOAD_CHECK(word, "x:");
    inFile >> quat.x;
    inFile >> word;
    LOAD_CHECK(word, "y:");
    inFile >> quat.y;
    inFile >> word;
    LOAD_CHECK(word, "z:");
    inFile >> quat.z;
    inFile >> word;
    LOAD_CHECK(word, "w:");
    inFile >> quat.w;
}

void readPose(std::ifstream& inFile, geometry_msgs::Pose& pose){
    std::string word;
    inFile >> word;
    LOAD_CHECK(word, "pose:");
    readPosition(inFile, pose.position);
    readOrientation(inFile, pose.orientation);
}

void savePath(const nav_msgs::Path& path, std::string filename){
    std::ofstream outFile{BASEDIR + filename};
    if (!outFile){
        throw std::runtime_error{"Could not save file " + filename};
    }
    
    outFile << path;
}

void loadPath(nav_msgs::Path& path, std::string filename){
    std::ifstream inFile{BASEDIR + filename};
    if (!inFile){
        throw std::runtime_error{"Could not open file " + filename};
    }
    std::string word;
    double dub;
    // get header:
    readHeader(inFile, path.header);
    inFile >> word;
    LOAD_CHECK(word, "poses[]");
    unsigned int poseCount = 0;
    while(inFile >> word){
        LOAD_CHECK(word, "poses[" + std::to_string(poseCount) + "]:");
        geometry_msgs::PoseStamped ps;
        readHeader(inFile,ps.header);
        readPose(inFile, ps.pose);
        path.poses.push_back(ps);
        poseCount++;
    }
    if (inFile.fail()){
        ROS_ERROR_STREAM("Error loading path from " << filename);
        throw std::runtime_error{"Error loading path from " + filename};
    }
}

}