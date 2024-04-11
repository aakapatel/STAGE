#ifndef GRAPH_MANAGEMENT_H
#define GRAPH_MANAGEMENT_H

#include "graph_utils.h"
#include "visualization_server.h"

class GraphManager
{

public:

  enum GraphStatus {
    OK = 0,                // Everything is OK as expected.
    READY,                 // Everything is OK as expected.
    ERR_KDTREE,            // Could not search nearest neigbors from kdtree.
    ERR_NO_FEASIBLE_PATH,  // Could not find any path.
    NO_GAIN,               // No non-zero gain found.
    NOT_OK                // Any other errors.
  };
  
  enum PlannerMode {
    EXPLORE = 0,
    RETURN_HOME
  };

  GraphManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, 
               MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel> *map_manager);
  
  void reset();
  void initializeAttributes ();
  void loadParams();
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params,
                       const BoundedSpaceParams& local_space_params);
  void setState(StateVec& state);
  void setGlobalFrame(std::string frame_id);
  void setExplorationDirection (geometry_msgs::PoseStamped pose);
  Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond q);
  bool sampleState(StateVec root_state, StateVec& vertex);
  bool sampleStateToExpandGraph(StateVec root_state, StateVec& vertex);
  
  bool sampleVertex(Vertex& vertex);
  void expandGraph(std::shared_ptr<GraphUtils> graph_manager,
                   Vertex& new_vertex, ExpandGraphReport& rep,
                   bool allow_short_edge = false);
  GraphStatus buildGraph(std::shared_ptr<GraphUtils> graph);

  Eigen::Quaternion<float> Euler2Quaternion(Eigen::Vector3d v);

  bool extractShortestPaths (std::shared_ptr<GraphUtils> graph, ShortestPathsReport rep, Vertex* root, 
                             std::vector<std::vector<Vertex *>> &dijkstra_paths);

  double getRelativeAngle(Eigen::Vector3d point);

  StateVec eigenToState(Eigen::Vector3d v);


  bool modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
                  Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                  Eigen::Vector3d& p1_mod); 

  bool improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                       std::vector<geometry_msgs::Pose>& path_mod,
                       bool relaxed, bool edge_refinement);

  void segmentPath(nav_msgs::Path path,
                   nav_msgs::Path &path_seg);
  void improvePath(std::vector<Vertex *> raw_path, std::vector<Vertex *> &path);

  std_msgs::ColorRGBA heightMapColor(double h);


  void clusterNodes (std::shared_ptr<GraphUtils> g); 
  // GRAPH AND PLANNING
  std::vector<int> performShortestPathsClustering(
    const std::shared_ptr<GraphUtils> graph_manager,
    const ShortestPathsReport& graph_rep, std::vector<Vertex*>& vertices,
    double dist_threshold = 0.5, double principle_path_min_length = 0.5,
    bool refinement_enable = true);

  void resetLocalGraph();
  void resetPoseGraph();
  void resetProjectedGraph();
  void resetGlobalGraph();
  void resetPathWaysGraph();
  void resetSparseGraph();

  GraphManager::GraphStatus runLocalPlanner();
  GraphManager::GraphStatus runGlobalPlanner();
  GraphManager::GraphStatus planLocalPath(std::vector<std::vector<Vertex *>> dijkstra_paths);
  bool planGlobalPath();
  nav_msgs::Path getLocalExplorationPath();
  nav_msgs::Path getGlobalExplorationPath();


  void visualizeLocalGraph();

  void visualizeVoxelGain(std::shared_ptr<GraphUtils> graph);

  void visualizeGlobalGraph();

  void visualizeSparseGraph();

  void visualize();

  double getPathLength(std::vector<Vertex *> path);

  void visualizeLocalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> local_paths);

  void visualizeGlobalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> global_paths);
  void visualizeImprovedGlobalPaths(std::vector<std::vector<Vertex *>> global_path);

  void visualizePathWaysGraph();

  void visualizePoseGraph ();

  void updateGlobalGraph();
  void expandGlobalGraph(std::vector<std::vector<Vertex *>> dijkstra_paths);

  void computeVolumetricGainRayCasting(StateVec& state, VolumetricGain& vgain);

  void expandGraph (int iterations_max, Vertex* root, Vertex* goal, double radius);

  void setHomingPath();

  void visualizeRepositionPath();

  double projectSample(Eigen::Vector3d& sample,
                       MapManager::VoxelStatus& voxel_status);
  ProjectedEdgeStatus getProjectedEdgeStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
      std::vector<Eigen::Vector3d>& projected_edge, bool);

  int global_root_id = 0;

  ros::Timer graph_timer_;

  double average_gain;

  double truncate_sample_Z_ = 1;


  Eigen::Vector3d latest_root;
  Eigen::Vector3d previous_root;
  Eigen::Vector3d overlap_point;

  bool init = true;
  bool run_local_planner = true;

  Vertex * local_goal_vertex;
  Vertex* local_root_vertex;
  Vertex* latest_root_vertex;
  Vertex* prev_pose_vertex_ = nullptr;
  Vertex* home_vertex_ = nullptr;

  PlannerMode planner_mode_ = GraphManager::PlannerMode::EXPLORE;
  GraphStatus graph_status_ = GraphManager::GraphStatus::OK;
  


private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  MapManagerVoxblox<MapManagerVoxbloxServer,
  MapManagerVoxbloxVoxel> *map_manager_;
  
  VisualizationServer* visualization_server_;
  std::string sensor_name;
  SensorParams sensor_params_;
  SensorParams free_frustum_params_;
  RobotParams robot_params_;
  BoundedSpaceParams local_space_params_;
  BoundedSpaceParams global_space_params_;
  BoundedSpaceParams graph_expansion_local_params_;
  BoundedSpaceParams graph_expansion_global_params_;
  PlanningParams planning_params_;

  RandomSampler random_sampler_;  // x,y,z,yaw: for exploration purpose
  RandomSampler random_sampler_graph_expansion_;  // x,y,z,yaw: for exploration purpose
  std::vector<BoundedSpaceParams> no_gain_zones_;
  

  StateVec robotStateWithYaw_;
  StateVec robotState_;
  std::shared_ptr<GraphUtils> local_graph_;
  std::shared_ptr<GraphUtils> projected_graph_;
  std::shared_ptr<GraphUtils> global_graph_;
  std::shared_ptr<GraphUtils> path_ways_graph_;
  std::shared_ptr<GraphUtils> improvement_graph_;
  std::shared_ptr<GraphUtils> pose_graph_;
  std::shared_ptr<GraphUtils> sparse_graph_;

  std::shared_ptr<GeofenceManager> geofence_manager_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> obs_pcl_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> feasible_corridor_pcl_;

  std::shared_ptr<SampleStatistic> stat_, stat_chrono_;
  std::vector<std::shared_ptr<GraphUtils>> graph_vec_;

  std::vector<Vertex *> state_history;

  kdtree *kd_tree_;

  ShortestPathsReport local_graph_rep_;
  ShortestPathsReport global_graph_rep_;
  ShortestPathsReport homing_graph_rep_;
  ShortestPathsReport improvement_graph_rep_;
  ShortestPathsReport path_ways_graph_rep_;
  ShortestPathsReport pose_graph_rep_;
  ShortestPathsReport sparse_graph_rep_;

  //boolean  
  bool odometry_ready = false;
  bool local_planner_status_ = true;
  bool global_planner_status_ = false;
  bool run_global_planner_ = false;
  bool run_homing_planner_ = true;
  bool reached_home_ = false;
  bool exploration_ongoing_ = false;
  bool local_exploration_ongoing_ = false;
  bool global_exploration_ongoing_ = false;
  

  int vertex_id_ = -1;
  int subgraph_ind_ = -1;
  int local_target_vertex_id;
  int re_plan_attempt = 0;
  std::vector<std::vector<double>> edge_inclinations_;

  int planning_iteration_ = -1;

  int temp_id = -1;
  int fake_id_count = -1;
  int latest_local_graph_id = 0;
  
  
  std::vector<std::vector<Vertex *>> local_dijkstra_paths_;
  std::vector<std::vector<Vertex *>> global_dijkstra_paths_;
  std::vector<std::pair<std::vector<Vertex *>, double>> local_paths_;
  std::vector<std::pair<std::vector<Vertex *>, double>> global_paths_;

  std::vector<int> global_vertices_ids_;
  std::vector<int> pose_graph_ids_;
  Vertex * global_root_vertex;

  std::vector<StateVec> sampled_states_;
  std::list<Vertex *> all_vertices;
  std::list<Vertex *> path_way_vertices;

  std::list<Vertex *> sparse_vertices;
  // variables

  Eigen::Vector3d robot_box_size_;
  Eigen::Vector3d safety_margin_;
  Eigen::Vector3d exploration_direction_;
  int num_edges_max_;
  double knn_range_ = 4;

  bool build_global_graph_ = false;

  // Global re-positioning in presence of unknown change in environment variables

  bool reached_end_of_local_path_ = true;
  bool reached_end_of_global_path_ = true;
  bool initialize_planner_ = true;
  int local_path_iterator_ = 0;
  int global_path_iterator_ = 0;
  std::vector<Vertex *> best_local_path_;
  std::vector<Vertex *> best_global_path_;



  bool reached_end_of_path_ = true;
  int path_iterator_ = 0;
  std::vector<Vertex *> best_path_;
  std::vector<Vertex *> homing_dijkstra_path_;
  nav_msgs::Path local_exploration_path_;
  nav_msgs::Path global_exploration_path_;
  nav_msgs::Path homing_path_;
  nav_msgs::Path improved_homing_path_;

  //


  std::string robot_type_ = "kAerial";
  Eigen::Quaterniond quat_;
  bool firstTime = true;
  double bounding_box_size_;
  double m_local_fov;
  bool map_received;
  bool reset_global_graph = true;
  bool m_max_allowed_Z;
  double obb_radius_;
  int mapDepth_;
  double m_volumetric_gain_threashold;
  bool check = false;
  std::vector<Eigen::Vector3d> local_exploration_targets_;
  std::vector<Eigen::Vector3d> global_repos_targets_;
  std::vector<Eigen::Vector3d> exploration_targets_;
  int iterations_local_;
  int iterations_global_;
  double max_edge_length_;
  bool m_reserved_graph;
  bool local_exploration;
  bool global_exploration;
  bool run_global_planner = false;
  double yaw_;
  Eigen::Vector3d exploration_target;
  Eigen::Vector3d home_position;

  geometry_msgs::PoseStamped exploration_dir_pose;
  nav_msgs::Path exploration_path_;

  // ros::NodeHandle m_nh;
  std::string world_frame_id; // the map frame
  std::string m_odometry;
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  nav_msgs::Odometry m_currentPose;
  nav_msgs::Odometry m_prevPose; // Holds the point where the direction changed.
  std::vector<Eigen::Vector3d> visited_exploration_targets;

  // Subscriber
  ros::Subscriber m_poseSubscriber;
  ros::Subscriber m_goalSubscriber;
  // message_filters::Subscriber<nav_msgs::Odometry> *m_poseSubscriber;

  // Publisher
  ros::Publisher m_modified_Path_Pub;
  ros::Publisher homing_Path_Pub;
  ros::Publisher m_frontierSelectionPub;
  ros::Publisher command_pub;
  ros::Publisher local_Paths_Vis_Pub_;
  ros::Publisher voxel_Gain_Vis_Pub_;
  ros::Publisher global_Paths_Vis_Pub_;
  ros::Publisher improved_global_Paths_Vis_Pub_;
  ros::Publisher reposition_Path_Vis_Pub_;
  ros::Publisher local_Graph_Vis_Pub_;
  ros::Publisher global_Graph_Vis_Pub_;
  ros::Publisher path_Ways_Graph_Vis_Pub_;
  ros::Publisher pose_Graph_Vis_Pub_;
  ros::Publisher overlap_Vis_Pub_;
  ros::Publisher random_Nodes_Vis_Pub_;
  ros::Publisher sparse_Graph_Vis_Pub_;
  ros::Publisher tree_Vis_Pub_;
  ros::Publisher execution_Time_Pub_;
  ros::Publisher exploration_Direction_Pub;

  // Services
  ros::ServiceServer m_exploration_path_srv;
};

#endif
