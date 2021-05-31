#include "path_server/PathServer.h"

/**
 * 
 * 
 * make event instead of updatefunc!!!!
 * 
**/

int main(int argc, char** argv){
    ros::init(argc, argv, "path_server");
    ros::NodeHandle nh{"~"};
    PathServer pathServer{&nh};
    bool finished = false;
    ros::Rate loopRate{30};
    while(ros::ok()){
        finished = pathServer.update();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}