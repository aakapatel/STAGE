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
  void reset();
  void periodicTimerCallback(const ros::TimerEvent& event);


 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
      map_manager_;

  std::vector<std::string> robots_list_;
  ros::Timer periodic_timer_;

  ros::ServiceClient planner_client_share_graph;



  VisualizationServer* central_visualization_server_;
  StateVec robotState_;
  std::shared_ptr<GraphUtils> central_graph_;

  ShortestPathsReport central_graph_rep_;
};

#endif
