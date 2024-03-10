#ifndef GRAPH_MANAGEMENT_H
#define GRAPH_MANAGEMENT_H

#include "graph_utils.h"


struct node
{

  Eigen::Vector3d *point;
  node *myParent = nullptr;
  double distanceToParent = -1;
  std::vector<node *> myChilds{};
  std::vector<node *> myPath{};
  double distanceToGoal;
  node(float x, float y, float z)
  {
    point = new Eigen::Vector3d(x, y, z);
  }
  node(Eigen::Vector3d *givenPoint)
  {
    point = new Eigen::Vector3d(givenPoint->x(), givenPoint->y(), givenPoint->z());
  }
  Eigen::Vector3d *getNodePointer();
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
  GraphManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, 
               MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel> *map_manager);

  virtual void odomCallback(const nav_msgs::Odometry::ConstPtr &pos);
  Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond q);
  double calculateDistance(StateVec s1, StateVec s2);
  void sampleNodes(std::vector<node *> &nodes, StateVec state, int iterations_max, double bounding_box_size);
  std::vector<Eigen::Vector3d> pointsToEigen(std::vector<geometry_msgs::Point> points);
  bool nextExplorationPath(stage_planner::explorationPath::Request &req,
                           stage_planner::explorationPath::Response &res);
  Eigen::Quaternion<float> Euler2Quaternion(Eigen::Vector3d v);

  bool extractShortestPaths (std::shared_ptr<GraphUtils> graph, ShortestPathsReport rep, Vertex* root, 
                             std::vector<std::vector<Vertex *>> &dijkstra_paths);
  double calculateAngle(Eigen::Vector3d fpoint);

  bool isLocal(Eigen::Vector3d point);
  
  void orientVertices (std::shared_ptr<GraphUtils> graph, const ShortestPathsReport& rep); 
  
  StateVec eigenToState(Eigen::Vector3d v);


  bool modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
                  Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                  Eigen::Vector3d& p1_mod); 

  bool improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                       std::vector<geometry_msgs::Pose>& path_mod,
                       bool relaxed);

  void segmentPath(nav_msgs::Path path,
                   nav_msgs::Path &path_seg);
  void improvePath(std::vector<Vertex *> raw_path, std::vector<Vertex *> &path);

  std_msgs::ColorRGBA heightMapColor(double h);

  void printVoxelStatus(MapManager::VoxelStatus vs);

  bool pointInsideFrustumVolume (StateVec state);

  void extractVoxelsInsideFrustum (StateVec state, 
                                   std::vector<Eigen::Vector3d> &free,
                                   std::vector<Eigen::Vector3d> &occupied,
                                   std::vector<Eigen::Vector3d> &unknown); 
  void clusterNodes (std::shared_ptr<GraphUtils> g); 
  // GRAPH AND PLANNING
  
  void resetLocalGraph();
  void resetGlobalGraph();
  void resetPathWaysGraph();
  void resetSparseGraph();

  bool buildLocalGraph();
  bool planLocalPath(std::vector<std::vector<Vertex *>> dijkstra_paths);

  bool planGlobalPath(std::vector<std::vector<Vertex *>> dijkstra_paths);
  
  void visualizeLocalGraph();
  
    void visualizeTree (std::vector<node *> RRT_TREE);

  void visualizeGlobalGraph();
  
  void visualizeSparseGraph();

  void visualize();

  double getPathLength(std::vector<Vertex *> path);

  void visualizeLocalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> local_paths);

  void visualizeGlobalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> global_paths);
  void visualizeImprovedGlobalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> global_paths);

  void visualizePathWaysGraph();
  
  void visualizePoseGraph ();

  bool getNeighbours(const StateVec *state, double range, std::vector<Vertex *> *v_res);
  void insertVertexKdtree(Vertex *v);
  void resetKdtree();

  int assignVertexID();

  void updateKdtree(std::shared_ptr<GraphUtils> graph);

  void updateGraph(const ros::TimerEvent &event);

  bool setLocalPath();
  bool setGlobalPath();

  double computeExplorationGain (StateVec v);
 
  void computeVolumetricGainRayCasting(StateVec& state, VolumetricGain& vgain);
  
  void expandGraph (int iterations_max, Vertex* root, Vertex* goal, double radius);
  
  bool expandSparseGraph();
  void updateSparseGraph();
  
  void insertBestPathsToSparseGraph (std::vector<std::vector<Vertex *>> dijkstra_paths); 
  
  bool setPath();

  void visualizeRepositionPath();


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
  


private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  std::string sensor_name;
  SensorParams sensor_params_;
  SensorParams free_frustum_params_;
  RobotParams robot_params_;
  BoundedSpaceParams local_space_params_;
  BoundedSpaceParams global_space_params_;
  PlanningParams planning_params_;
  
  std::vector<BoundedSpaceParams> no_gain_zones_;
  StateVec robotStateWithYaw_;
  StateVec robotState_;
  std::shared_ptr<GraphUtils> sub_graph_;
  std::shared_ptr<GraphUtils> global_graph_;
  std::shared_ptr<GraphUtils> path_ways_graph_;
  std::shared_ptr<GraphUtils> improvement_graph_;
  std::shared_ptr<GraphUtils> pose_graph_;
  std::shared_ptr<GraphUtils> sparse_graph_;
   
  std::shared_ptr<GeofenceManager> geofence_manager_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> obs_pcl_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> feasible_corridor_pcl_;
  
  std::vector<std::shared_ptr<GraphUtils>> graph_vec_;
  
  std::vector<Vertex *> state_history;
    
  MapManagerVoxblox<MapManagerVoxbloxServer,
                    MapManagerVoxbloxVoxel> *map_manager_;
  kdtree *kd_tree_;

  ShortestPathsReport sub_graph_rep_;
  ShortestPathsReport global_graph_rep_;
  ShortestPathsReport homing_graph_rep_;
  ShortestPathsReport improvement_graph_rep_;
  ShortestPathsReport path_ways_graph_rep_;
  ShortestPathsReport pose_graph_rep_;
  ShortestPathsReport sparse_graph_rep_;
  
  int vertex_id_ = -1;
  int subgraph_ind_ = -1;
  int local_target_vertex_id;
  int re_plan_attempt = 0;

  int planning_iteration_ = -1;

  int temp_id = -1;
  int fake_id_count = -1;
  int latest_sub_graph_id = 0;

  std::vector<std::pair<std::vector<Vertex *>, double>> local_paths_;
  std::vector<std::pair<std::vector<Vertex *>, double>> global_paths_;

  std::vector<int> global_vertices_ids_;
  std::vector<int> pose_graph_ids_;
  Vertex * global_root_vertex;

  std::list<Vertex *> all_vertices;
  std::list<Vertex *> path_way_vertices;

  std::list<Vertex *> sparse_vertices;
  // variables
  
  std::vector<double> safety_margin;
  std::vector<double> robot_box_size;
  Eigen::Vector3d robot_box_size_;
  Eigen::Vector3d safety_margin_;
  int num_edges_max_;
  double knn_range_ = 4;

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
  nav_msgs::Path mod_path_;

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
  float SCALER_X, SCALER_Y, SCALER_Z, highest_x, highest_y,
      highest_z, lowest_x, lowest_y, lowest_z;
  double m_min_dist_bw_nodes;
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
  nav_msgs::Path homing_path_;
  Eigen::Vector3d home_position;

  nav_msgs::Path exploration_path_;

  // ros::NodeHandle m_nh;
  std::string m_worldFrameId; // the map frame
  std::string m_odometry;
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  nav_msgs::Odometry m_currentPose;
  nav_msgs::Odometry m_prevPose; // Holds the point where the direction changed.
  std::vector<Eigen::Vector3d> visited_exploration_targets;

  // Subscriber
  ros::Subscriber m_poseSubscriber;

  // message_filters::Subscriber<nav_msgs::Odometry> *m_poseSubscriber;

  // Publisher
  ros::Publisher m_modified_Path_Pub;
  ros::Publisher m_frontierSelectionPub;
  ros::Publisher command_pub;
  ros::Publisher local_Paths_Vis_Pub_;
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


    // Services
  ros::ServiceServer m_exploration_path_srv;
};

#endif
