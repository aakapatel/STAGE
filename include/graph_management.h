#ifndef GRAPH_MANAGEMENT_H
#define GRAPH_MANAGEMENT_H

#include "frontiers_extraction.h"
#include "graph.h"
#include <kdtree/kdtree.h>

struct Vertex {
  Vertex(int v_id, State v_state) {
    id = v_id;
    state << v_state[0], v_state[1], v_state[2], v_state[3];
    parent = NULL;
    distance = 0;
    gain = 0;
  }

  // Unique id for each vertex (root: 0, others: positive number).
  int id;
  // State of the vertex: x, y, z, yaw.
  State state;
  // NBVP legacy, keeping for now if wants to build a tree to compare.
  Vertex* parent;
  std::vector<Vertex*> children;
  // Distance to root.
  double distance;
  // volumetric gain 
  double gain;

};

class GraphManager
{

public:
  GraphManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  virtual void odomCallback(const nav_msgs::Odometry::ConstPtr &pos);
  Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond q);
  virtual void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &map);
  double calculateDistance(ufo::math::Vector3 point1, ufo::math::Vector3 point2);
  bool isInCollision(ufo::map::OccupancyMapColor const &map,
                     ufo::geometry::BoundingVar const &bounding_volume,
                     bool occupied_space, bool free_space,
                     bool unknown_space, ufo::map::DepthType min_depth);
  ufo::geometry::OBB makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius);
  double computeVolumetricGain(ufo::math::Vector3 frontier, ufo::map::OccupancyMapColor const &map);
  ufo::math::Vector3 tunePlanningSpace ();
  void sampleNodes (std::vector<ufo::math::Vector3>& nodes); 
  ufo::math::Vector3 pointToUfo(geometry_msgs::Point p);
  std::vector<ufo::math::Vector3> pointsToUfo(std::vector<geometry_msgs::Point> points);
  void fetchExplorationTargets();
  bool isLastTargetFrontier();
  bool nextExplorationPath (stage_planner::explorationPath::Request &req,
                             stage_planner::explorationPath::Response &res);
  Eigen::Quaternion<float> Euler2Quaternion(ufo::math::Vector3 v);


  // GRAPH AND PLANNING 
  
  void buildGraph ();

  // GRAPH UTILS
  
  // Initialize a fresh graph.
  void reset();
  
  double distanceBetweenVertices (Vertex* v, Vertex* u);

  // Could change an index to break down the graph into sug-graphs.
  int generateSubgraphIndex();
  // Generate ID for new vertex.
  int generateVertexID();

  // Basic functions on graph including add new vertex and edge.
  void addVertex(Vertex* v);
  void addEdge(Vertex* v, Vertex* u, double weight);
  void removeEdge(Vertex* v, Vertex* u);

  int getNumVertices() { return graph_->getNumVertices(); }
  int getNumEdges() { return graph_->getNumEdges(); }

  Vertex* getVertex(int id) { return vertices_map_[id]; }

  bool findShortestPaths(ShortestPathsReport& rep);
  bool findShortestPaths(int source_id, ShortestPathsReport& rep);

  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order, std::vector<int>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order, std::vector<Vertex*>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order,
                       std::vector<Eigen::Vector3d>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order,
                       std::vector<State>& path);
  double getShortestDistance(int target_id, const ShortestPathsReport& rep);
  int getParentIDFromShortestPath(int target_id,
                                  const ShortestPathsReport& rep);

  // Nearest neigbor lookup.
  bool getNearestVertex(const State* state, Vertex** v_res);
  bool getNearestVertexInRange(const State* state, double range,
                               Vertex** v_res);
  bool getNearestVertices(const State* state, double range,
                          std::vector<Vertex*>* v_res);



  // variables
  
  // Boost graph 
  std::shared_ptr<Graph> graph_;
  // Mapping from vertex id to vertex property.
  std::unordered_map<int, Vertex*> vertices_map_;
  // id:  <neighbor id, edge cost>
  std::map<int, std::vector<std::pair<int, double>>> edge_map_;  

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // IDs are non-negative integer from 0 (root node)
  // and increased as adding new sub-graph or new vertices.
  int subgraph_ind_;
  int id_count_ = 0;
  
  // Kd-tree for nearest neigbor lookup, also keep all vertices.
  kdtree* kd_tree_;

  // Map from local id for Boost Graph Lib to global ID including
  // <sub-graph-id,vertex-id>. This is mainly for debug purpose.
  std::unordered_map<int, std::pair<int, int>> local_id_map_;

  // variables
  State robotState_;
  Eigen::Quaterniond quat_;
  bool firstTime = true;
  ufo::math::Vector3 m_currentPosition;
  double m_currentHeading;
  double bounding_box_size_, m_global_space;
  double m_local_fov;
  bool map_received;
  bool initialize_global_graph = true;
  std::string m_ufomap;
  bool m_max_allowed_Z;
  double obb_radius_;
  int mapDepth_;
  double m_volumetric_gain_threashold;
  bool check = false;
  float SCALER_X, SCALER_Y, SCALER_Z, highest_x, highest_y, highest_z, lowest_x, lowest_y, lowest_z;
  double m_min_dist_bw_nodes;
  ufo::math::Vector3 m_local_exploration_target;
  std::vector<ufo::math::Vector3> m_global_repos_targets;
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
