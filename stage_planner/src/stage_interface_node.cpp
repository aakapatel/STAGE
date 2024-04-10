#include <ros/ros.h>
#include "stage_interface.h"
#include "stage_interface.cpp"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stage_Interface_Node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>* map_manager = 
    new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel> (nh, nh_private);   

  StageInterface stageInterface(nh, nh_private, map_manager);


  ros::spin();

  // try
  // {
  //     ros::spin();
  // }
  // catch (std::runtime_error &e)
  // {
  //     ROS_ERROR("Interface Error: %s", e.what());
  //     return -1;
  // }

  return 0;
}
