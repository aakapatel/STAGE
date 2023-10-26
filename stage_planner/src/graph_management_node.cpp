#include <ros/ros.h>
#include "graph_management.h"
#include "graph_management.cpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Graph_Management_Node");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>* map_manager = 
        new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel> (nh, nh_private);   
    GraphManager graphManager(nh, nh_private, map_manager);
    
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
