#include <iostream>

#include "path_tracking/PathTracker.h"

using namespace std;

int main(int argc, char** argv){
    cout << "main.cpp\n";
    ros::init(argc, argv, "path_tracking");
    string throttleTopic = "throttle_cmd";
    string brakeTopic = "brake_cmd";
    string steeringTopic = "steering_cmd";
    string odomTopic = "odom";
    string pathTopic = "path";

    ros::Rate loopRate = 10;
    while(ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}