#include <iostream>

#include "path_tracking/PathTracker.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "path_tracking");
    ros::NodeHandle pn{"~"};

    Stanley stan{&pn};
    ros::Rate loopRate = 40;
    while(ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}