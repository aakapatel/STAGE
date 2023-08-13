#ifndef GRAPH_MANAGEMENT_H
#define GRAPH_MANAGEMENT_H

#include "frontiers_extraction.h"
#include <omp.h>

class GraphManager
{

public:
  GraphManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);



  virtual void updateCurrentPosition(const nav_msgs::Odometry::ConstPtr &pos);

  virtual void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &map);

  double calculateDistance(ufo::math::Vector3 point1, ufo::math::Vector3 point2);

  bool isInCollision(ufo::map::OccupancyMapColor const &map,
                     ufo::geometry::BoundingVar const &bounding_volume,
                     bool occupied_space, bool free_space,
                     bool unknown_space, ufo::map::DepthType min_depth);

  ufo::geometry::OBB makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius);

  double calculateAngle(ufo::math::Vector3 fpoint, ufo::math::Vector3 position, double heading);

  bool isLocal(ufo::math::Vector3 point);

  ufo::math::Vector3 fetchCurrentPosition();

  double computeVolumetricGain(ufo::math::Vector3 frontier, ufo::map::OccupancyMapColor const &map);

  ufo::math::Vector3 tunePlanningSpace(float range, ufo::math::Vector3 root_node,
                                       ufo::map::OccupancyMapColor const &map);

  ufo::math::Vector3 pointToUfo(geometry_msgs::Point p);

  std::vector<ufo::math::Vector3> pointsToUfo(std::vector<geometry_msgs::Point> points);

  void fetchExplorationTargets();

  bool isLastTargetFrontier();

  bool nextExplorationPath (stage_planner::explorationPath::Request &req,
                             stage_planner::explorationPath::Response &res);
  
  Eigen::Quaternion<float> Euler2Quaternion(ufo::math::Vector3 v);

private:


  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;


  // variables
  bool firstTime = true;
  ufo::math::Vector3 m_currentPosition;
  double m_currentHeading;
  double m_local_space, m_global_space;
  double m_local_fov;
  bool map_received;
  bool initialize_global_graph = true;
  std::string m_ufomap;
  bool m_max_allowed_Z;
  double m_obb_radius;
  int m_planning_depth;
  double m_volumetric_gain_threashold;
  bool check = false;
  float SCALER_X, SCALER_Y, SCALER_Z, highest_x, highest_y, highest_z, lowest_x, lowest_y, lowest_z;
  double m_min_dist_bw_nodes;
  ufo::math::Vector3 m_local_exploration_target;
  std::vector<ufo::math::Vector3> m_global_repos_targets;
  int m_iterations_local;
  int m_iterations_global;
  double m_max_edge_length;
  bool m_reserved_graph;
  bool local_exploration;
  bool global_exploration;
  bool run_global_planner = false;
  ufo::math::Vector3 exploration_target;
  nav_msgs::Path homing_path_;
  ufo::math::Vector3 home_position;
  
  nav_msgs::Path exploration_path_to_publish_;

  
  // ros::NodeHandle m_nh;
  std::string m_worldFrameId; // the map frame
  std::string m_odometry;
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  nav_msgs::Odometry m_currentPose;
  nav_msgs::Odometry m_prevPose; // Holds the point where the direction changed.
  std::vector<ufo::math::Vector3> visited_exploration_targets;
  
    
  // Subscriber
  ros::Subscriber m_poseSubscriber;
  ros::Subscriber m_ufoMapSubscriber;

  // message_filters::Subscriber<nav_msgs::Odometry> *m_poseSubscriber;
  // message_filters::Subscriber<ufomap_msgs::UFOMapStamped> *m_ufoMapSubscriber;
 
  // Publisher
  ros::Publisher m_frontierPub;
  ros::Publisher m_frontierSelectionPub;
  ros::Publisher command_pub;

  //Services 
  ros::ServiceClient m_frontier_client;
  ros::ServiceClient m_target_query_client;
  ros::ServiceServer m_exploration_path_srv;

  };

#endif
