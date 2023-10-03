#include <ros/ros.h>
#include "frontiers_extraction.h"
#include "frontiers_extraction.cpp" 

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Frontier_Extraction_Node");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>* map_manager = 
        new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel> (nh, nh_private);   
    FrontierExtraction frontierExtraction(nh, nh_private, map_manager);

    ros::spin();
    // try
    // {
    //     ros::spin();
    // }
    // catch (std::runtime_error &e)
    // {
    //     ROS_ERROR("Graph Building Error: %s", e.what());
    //     return -1;
    // }

    return 0;
}
