#include <ros/ros.h>
#include "frontiers_extraction.h"
#include "frontiers_extraction.cpp" 

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Frontier_Extraction_Node");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    FrontierExtraction frontierExtraction(nh, nh_private);

    ros::spinOnce();
    try
    {
        ros::spin();
    }
    catch (std::runtime_error &e)
    {
        ROS_ERROR("octree exception: %s", e.what());
        return -1;
    }

    return 0;
}
