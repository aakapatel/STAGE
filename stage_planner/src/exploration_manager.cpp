#include "exploration_manager.h"

ExplorationManager::ExplorationManager(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,

    MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
        map_manager)
    : nh_(nh),
      nh_private_(nh_private),
      map_manager_(map_manager)

{
  // Initialize attributes
  ExplorationManager::initializeAttributes();
};

void ExplorationManager::initializeAttributes() {
  std::vector<std::string> parse_str_list;
  std::string param_name;
  std::string ns = ros::this_node::getName();
  param_name = ns + "/robots_list";
  ros::param::get(param_name, parse_str_list);
  if (parse_str_list.size() <= 0) {
    ROSPARAM_WARN(param_name, "");
  } else {
    robots_list_ = parse_str_list;
    std::string str_tmp = "Available robots : ";
    for (int i = 0; i < robots_list_.size(); ++i) {
      str_tmp += robots_list_[i] + ", ";
    }
    ROSPARAM_INFO(str_tmp);
  }
  std::string frame_id = ns + "/world_frame";

  reset();

  central_visualization_server_ = new VisualizationServer(nh_, nh_private_);
  central_visualization_server_->setGlobalFrame(frame_id);

  // Ros services

  planner_client_share_graph =
      nh_.serviceClient<stage_planner::shareGraph>("/share_graph");

  // Periodic timer

  periodic_timer_ = nh_.createTimer(
      ros::Duration(60), &ExplorationManager::periodicTimerCallback, this);
}

void ExplorationManager::reset() {
  if (central_graph_ != NULL) {
    central_graph_->reset();
  }
  central_graph_.reset(new GraphUtils());
  central_graph_rep_.reset();
}

void ExplorationManager::periodicTimerCallback(const ros::TimerEvent& event) {
  for (int i = 0; i < robots_list_.size(); ++i) {
    stage_planner::shareGraph share_graph_srv;
    std::string robot_name = robots_list_[i];
    stage_planner::Graph g;

    share_graph_srv.request.robot_name = robot_name;

    if (planner_client_share_graph.call(share_graph_srv)) {
      g = share_graph_srv.response.graph;
      central_graph_->convertMsgToGraph(g);
      central_visualization_server_->visualizeCentralGraph(central_graph_);

    } else {
      ROS_WARN("Share graph service failed \n");
    }
  }
}
