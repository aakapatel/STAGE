#ifndef EXPLORATION_MANAGER_H
#define EXPLORATION_MANAGER_H

#include "stage_interface.h"

class ExplorationManager {
 public:
  ExplorationManager(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private,
                     MapManagerVoxblox<MapManagerVoxbloxServer,
                                       MapManagerVoxbloxVoxel>* map_manager);
  
  void initializeAttributes();



 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
      map_manager_;

  std::vector<std::string> robots_list_;





  VisualizationServer* visualization_server_;
  StateVec robotState_;
  std::shared_ptr<GraphUtils> centeral_graph_;

  ShortestPathsReport centeral_graph_rep_;
};

#endif
