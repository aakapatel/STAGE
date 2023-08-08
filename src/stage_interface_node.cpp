#include <ros/ros.h>
#include "stage_interface.h"
#include "stage_interface.cpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stage_Interface_Node");
    const ros::NodeHandle &private_nh = ros::NodeHandle("~");

    StageInterface stageInterface;
    
    stageInterface.run_planner();

    ros::spinOnce();

    try
    {
        ros::spin();
    }
    catch (std::runtime_error &e)
    {
        ROS_ERROR("Interface Error: %s", e.what());
        return -1;
    }

    return 0;
}
