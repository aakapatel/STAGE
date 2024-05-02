#include <ros/ros.h>

#include "exploration_manager.cpp"
#include "exploration_manager.h"

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "Exploration_Manager_Node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  MapManagerVoxblox<MapManagerVoxbloxServer,
                    MapManagerVoxbloxVoxel>* map_manager =
      new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>(
          nh, nh_private);

  ExplorationManager explorationManager(nh, nh_private, map_manager);

  ros::spin();


  return 0;
}
