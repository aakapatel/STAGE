#include <ros/ros.h>
#include "graph_management.h"
#include "graph_management.cpp"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Graph_Management_Node");
    const ros::NodeHandle& private_nh = ros::NodeHandle("~");

    GraphManager graphManager;
    
    ros::spinOnce();
    try{
        ros::spin();
   

    }catch(std::runtime_error& e){
        ROS_ERROR("Graph Building Error: %s", e.what());
        return -1;
    }

    return 0;
}
