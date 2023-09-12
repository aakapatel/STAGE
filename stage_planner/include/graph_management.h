#ifndef GRAPH_MANAGEMENT_H
#define GRAPH_MANAGEMENT_H

#include "frontiers_extraction.h"
#include "graph_utils.h"

#include "map_manager.h"
#include "map_manager_voxblox_impl.h"
#include "params.h"

struct node
{

  ufo::math::Vector3 *point;
  node *myParent = nullptr;
  double distanceToParent = -1;
  std::vector<node *> myChilds{};
  std::vector<node *> myPath{};
  double distanceToGoal;
  node(float x, float y, float z)
  {
    point = new ufo::math::Vector3(x, y, z);
  }
  node(ufo::math::Vector3 *givenPoint)
  {
    point = new ufo::math::Vector3(givenPoint->x(), givenPoint->y(), givenPoint->z());
  }
  ufo::math::Vector3 *getNodePointer();
  void addChild(node *newChild)
  {
    myChilds.push_back(newChild);
  }
  void addParent(node *newParent)
  {
    myParent = newParent;
  }
  void readyForDeletion()
  {
    delete point;
  }
};

class GraphManager
{

public:
  GraphManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel> *map_manager);

  virtual void odomCallback(const nav_msgs::Odometry::ConstPtr &pos);
  Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond q);
  virtual void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &map);
  double calculateDistance(ufo::math::Vector3 point1, ufo::math::Vector3 point2);
  bool isInCollision(ufo::map::OccupancyMapColor const &map,
                     ufo::geometry::BoundingVar const &bounding_volume,
                     bool occupied_space, bool free_space,
                     bool unknown_space, ufo::map::DepthType min_depth);
  ufo::geometry::OBB makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius);
  ufo::math::Vector3 tunePlanningSpace(State state, double bounding_box_size);
  void sampleNodes(std::vector<node *> &nodes, State state, int iterations_max, double bounding_box_size);
  void sampleNodesOBB (std::vector<node *>* nodes, int iterations_max, State root, State goal, double radius);
  std::pair<Eigen::Vector3d, Eigen::Vector3d> tuneBounds (State root, State goal, double radius);
  ufo::math::Vector3 pointToUfo(geometry_msgs::Point p);
  std::vector<ufo::math::Vector3> pointsToUfo(std::vector<geometry_msgs::Point> points);
  void fetchExplorationTargets();
  bool isLastTargetFrontier(geometry_msgs::Point last_target_frontier);
  bool nextExplorationPath(stage_planner::explorationPath::Request &req,
                           stage_planner::explorationPath::Response &res);
  Eigen::Quaternion<float> Euler2Quaternion(ufo::math::Vector3 v);

  State ufoToState(ufo::math::Vector3 v);
  ufo::math::Vector3 stateToUfo(State s);

  void segmentPath(std::vector<Vertex *> path,
                   nav_msgs::Path &path_seg);

  void improvePath(std::vector<Vertex *> raw_path, std::vector<Vertex *> &path);

  std_msgs::ColorRGBA heightMapColor(double h);

  void printVoxelStatus(MapManager::VoxelStatus vs);

  // GRAPH AND PLANNING

  void resetLocalGraph();
  void resetGlobalGraph();
  void resetFrontierGraph();
  void resetDebugGraph();

  void buildLocalGraph();
  
  void planLocalPath();

  void planGlobalPath();

  void visualizeLocalGraph();

  void visualizeGlobalGraph();

  void visualize();

  double getPathLength(std::vector<Vertex *> path);

  void visualizeLocalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> local_paths);

  void visualizeGlobalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> global_paths);

  void visualizeFrontierGraph();
  
  void visualizePoseGraph ();

  void visualizeRandomNodes ();
  
  bool getNeighbours(const State *state, double range, std::vector<Vertex *> *v_res);
  void insertVertexKdtree(Vertex *v);
  void resetKdtree();

  int assignVertexID();

  void updateKdtree();

  void updateGraph(const ros::TimerEvent &event);

  bool setLocalPath();
  bool setGlobalPath();

  double computeExplorationGain (State v);
 
  void expandGraph (int iterations_max, Vertex* root, Vertex* goal, double radius);
  
  State generateRandomState (State root, State goal);
  
  void debugGraph(State root, State goal);
  
  void buildReservedGraph();
  void insertLatestSubGraph();
  
  bool setPath();

  void visualizeRepositionPath();

  int global_root_id = 0;

  ros::Timer graph_timer_;
  
  double average_gain;

  bool robot_aerial_ = false;
  double max_graph_height_ = 2;

  Eigen::Vector3d latest_root;
  Eigen::Vector3d previous_root;
  Eigen::Vector3d overlap_point;

  bool init = true;
  
  Vertex * local_goal_vertex;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::shared_ptr<GraphUtils> sub_graph_;
  std::shared_ptr<GraphUtils> global_graph_;
  std::shared_ptr<GraphUtils> frontier_graph_;
  std::shared_ptr<GraphUtils> improvement_graph_;
  std::shared_ptr<GraphUtils> pose_graph_;
  std::shared_ptr<GraphUtils> debug_graph_;
  
  std::vector<std::shared_ptr<GraphUtils>> graph_vec_;

  MapManagerVoxblox<MapManagerVoxbloxServer,
                    MapManagerVoxbloxVoxel> *map_manager_;
  kdtree *kd_tree_;

  ShortestPathsReport sub_graph_rep_;
  ShortestPathsReport global_graph_rep_;
  ShortestPathsReport homing_graph_rep_;
  ShortestPathsReport improvement_graph_rep_;
  ShortestPathsReport frontier_graph_rep_;
  ShortestPathsReport pose_graph_rep_;
  ShortestPathsReport debug_graph_rep_;

  int vertex_id_ = -1;
  int subgraph_ind_ = -1;
  int local_target_vertex_id;
  
  int planning_iteration_ = -1;

  int temp_id = -1;

  int latest_sub_graph_id = 0;

  std::vector<std::pair<std::vector<Vertex *>, double>> local_paths_;
  std::vector<std::pair<std::vector<Vertex *>, double>> global_paths_;

  std::list<int> global_vertices_ids_;

  std::list<Vertex *> all_vertices;

  std::list<Vertex *> sparse_vertices;
  // variables
  State robotState_;

  Eigen::Vector3d robot_box_size_;
  Eigen::Vector3d safety_margin_;

  double safety_margin;
  double robot_box_x;
  double robot_box_y;
  double robot_box_z;

  bool build_global_graph_ = false;
 
  // Global re-positioning in presence of unknown change in environment variables

  bool reached_end_of_local_path_ = true;
  bool reached_end_of_global_path_ = true;
  int local_path_iterator_ = 0;
  int global_path_iterator_ = 0;
  std::vector<Vertex *> best_local_path_;
  std::vector<Vertex *> best_global_path_;



  bool reached_end_of_path_ = true;
  int path_iterator_ = 0;
  std::vector<Vertex *> best_path_;


  //

  Eigen::Quaterniond quat_;
  bool firstTime = true;
  ufo::math::Vector3 m_currentPosition;
  double m_currentHeading;
  double bounding_box_size_, m_global_space;
  double m_local_fov;
  bool map_received;
  bool reset_global_graph = true;
  std::string m_ufomap;
  bool m_max_allowed_Z;
  double obb_radius_;
  int mapDepth_;
  double m_volumetric_gain_threashold;
  bool check = false;
  float SCALER_X, SCALER_Y, SCALER_Z, highest_x, highest_y,
      highest_z, lowest_x, lowest_y, lowest_z;
  double m_min_dist_bw_nodes;
  std::vector<ufo::math::Vector3> local_exploration_targets_;
  std::vector<ufo::math::Vector3> global_repos_targets_;
  int iterations_local_;
  int iterations_global_;
  double max_edge_length_;
  bool m_reserved_graph;
  bool local_exploration;
  bool global_exploration;
  bool run_global_planner = false;
  ufo::math::Vector3 exploration_target;
  nav_msgs::Path homing_path_;
  ufo::math::Vector3 home_position;

  nav_msgs::Path exploration_path_;

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
  ros::Publisher local_Paths_Vis_Pub_;
  ros::Publisher global_Paths_Vis_Pub_;
  ros::Publisher reposition_Path_Vis_Pub_;
  ros::Publisher local_Graph_Vis_Pub_;
  ros::Publisher global_Graph_Vis_Pub_;
  ros::Publisher frontier_Graph_Vis_Pub_;
  ros::Publisher pose_Graph_Vis_Pub_;
  ros::Publisher overlap_Vis_Pub_;
  ros::Publisher random_Nodes_Vis_Pub_;
  ros::Publisher debug_Graph_Vis_Pub_;


    // Services
  ros::ServiceClient m_frontier_client;
  ros::ServiceClient m_target_query_client;
  ros::ServiceServer m_exploration_path_srv;
};

#endif
