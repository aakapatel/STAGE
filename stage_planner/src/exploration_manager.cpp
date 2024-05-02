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

  for (auto i : robots_list_) {
    ROS_WARN_STREAM("Available robots : " << i << "\n");
  }

}
