#include <ros/ros.h>
#include "frontiers_extraction.h"
#include "frontiers_extraction.cpp"

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Frontier_Extraction_Node");
    const ros::NodeHandle& private_nh = ros::NodeHandle("~");

    FrontierExtraction frontierExtraction;
    // frontierExtraction.fetchGlobalRepositioningTargets();  
    ros::spinOnce();
    
    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("octree exception: %s", e.what());
        return -1;
    }

    return 0;
}
