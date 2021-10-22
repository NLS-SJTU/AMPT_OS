#include "Decision.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    if(argc < 2){
        for(int i=0; i<argc; ++i){
            cout<<"-----"<<argv[i]<<endl;
        }
        ROS_ERROR("[AMPT]usage: rosrun ampt ampt_main paramfile");
        return -1;
    }
    ros::init(argc, argv, "ampt");
    ros::NodeHandle nh;
    
    Decision deci(nh, argv[1]);
    ROS_INFO("[AMPT]init done.");
    deci.run();
    ROS_INFO("[AMPT]shut down.");
    return 0;
}
