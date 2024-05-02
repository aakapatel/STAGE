#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include <dlfcn.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <kdtree/kdtree.h>
#include <mav_msgs/Status.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <omp.h>
#include <pcl/common/distances.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <ros/message_traits.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "geofence_manager.h"
#include "graph.h"
#include "map_manager.h"
#include "map_manager_voxblox_impl.h"
#include "params.h"
#include "random_sampler.h"
#include <stage_planner/Edge.h>
#include <stage_planner/Graph.h>
#include <stage_planner/Vertex.h>
#include "stage_planner/explorationPath.h"
#include "stage_planner/frontierRequest.h"
#include "stage_planner/homePlanning.h"
#include "stage_planner/initializePlanning.h"
#include "stage_planner/isFrontier.h"
#include "stage_planner/stopPlanning.h"
#include "stage_planner/shareGraph.h"
#include "std_msgs/Int64.h"
#include "trajectory.h"


typedef Eigen::Vector3d State;  // x, y, z

struct VolumetricGain {
  VolumetricGain()
      : gain(0),
        accumulative_gain(0),
        unknown_percentage(0),
        num_unknown_voxels(0),
        num_free_voxels(0),
        num_occupied_voxels(0),
        num_unknown_surf_voxels(0) {}

  void reset() {
    gain = 0;
    accumulative_gain = 0;
    unknown_percentage = 0;
    num_unknown_voxels = 0;
    num_free_voxels = 0;
    num_occupied_voxels = 0;
    num_unknown_surf_voxels = 0;
    is_frontier = false;
    // is_evaluated = false;
    occupied_voxels.clear();
    free_voxels.clear();
    unknown_voxels.clear();
    unseen_voxel_hash_keys.clear();
  }

  double gain;
  double accumulative_gain;
  double unknown_percentage;
  int num_unknown_voxels;
  int num_free_voxels;
  int num_occupied_voxels;
  int num_unknown_surf_voxels;
  std::vector<Eigen::Vector3d> occupied_voxels;
  std::vector<Eigen::Vector3d> free_voxels;
  std::vector<Eigen::Vector3d> unknown_voxels;
  std::vector<std::size_t> unseen_voxel_hash_keys;

  bool is_frontier;
  // bool is_evaluated = !is_frontier;

  void printGain() {
    std::cout << "Gains: " << gain << ", " << num_unknown_voxels << ", "
              << num_occupied_voxels << ", " << num_free_voxels << std::endl;
    std::cout << "Unknown percentage : " << unknown_percentage << "\n";
  }
};

// main back bone of the graph object

struct SampleStatistic {
  SampleStatistic()
      : num_vertices_fail(0),
        num_edges_fail(0),
        build_graph_time(0),
        compute_exp_gain_time(0),
        shortest_path_time(0),
        evaluate_graph_time(0),
        total_time(0) {}
  StateVec current_state;
  int num_vertices_fail;
  int num_edges_fail;
  // Sampled state in free space, but got rejected due to collision edges.
  std::vector<std::vector<double>> edges_fail;
  double build_graph_time;
  double compute_exp_gain_time;
  double shortest_path_time;
  double evaluate_graph_time;
  double total_time;
  void init(StateVec& state) { current_state = state; }

  void printTime() {
    total_time = build_graph_time + compute_exp_gain_time + shortest_path_time +
                 evaluate_graph_time;
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                  "Time statistics:\n \
        Build graph    : %3.3f (s)\n \
        Compute gain   : %3.3f (s)\n \
        Dijkstra       : %3.3f (s)\n \
        Evaluate graph : %3.3f (s)\n \
        Total          : %3.3f (s)",
                  build_graph_time, compute_exp_gain_time, shortest_path_time,
                  evaluate_graph_time, total_time);
  }
  void printTime(std::string title) {
    total_time = build_graph_time + compute_exp_gain_time + shortest_path_time +
                 evaluate_graph_time;
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                  "Time statistics (%s):\n \
        Build graph    : %3.3f (ms)\n \
        Compute gain   : %3.3f (ms)\n \
        Dijkstra       : %3.3f (ms)\n \
        Evaluate graph : %3.3f (ms)\n \
        Total          : %3.3f (ms)",
                  title.c_str(), build_graph_time, compute_exp_gain_time,
                  shortest_path_time, evaluate_graph_time, total_time);
  }
};

struct Vertex {
  Vertex(int v_id, StateVec v_state) {
    id = v_id;
    state << v_state[0], v_state[1], v_state[2], v_state[3];
    state_3d << v_state[0], v_state[1], v_state[2];
    vol_gain.reset();
    parent = NULL;
    distance = 0;
    gain = 0;
    volumetric_gain = 0;
    edges = 0;
    is_root = false;
    is_leaf_vertex = true;
    is_evaluated = false;
    is_local = false;
    is_global = false;
    is_frontier = false;
    is_cluster_centroid = false;
    is_hanging = false;
    is_inside_frustum = false;
    cluster_id = -1;
    sub_id = 0;
    sub_graph_id = 0;
  }

  // Unique id for each vertex (root: 0, others: positive number).
  int id;
  // State of the vertex: x, y, z.
  StateVec state;
  Eigen::Vector3d state_3d;
  VolumetricGain vol_gain;
  // NBVP legacy, keeping for now if wants to build a tree to compare.
  Vertex* parent;
  std::vector<Vertex*> children;
  // Distance to root.
  double distance;
  // exploration gain
  double gain;
  // volumetric gain
  double volumetric_gain;
  // number of edges
  int edges;
  // If it is root vertex
  bool is_root;
  // If node is randomly sampled
  bool is_leaf_vertex;
  // If a vertex is already evaluated to be a frontier
  bool is_evaluated;
  // If in local FOV
  bool is_local;
  // If a node is outside FOV
  bool is_global;
  // If a node itself is a frontier
  bool is_frontier;
  // if a vertex is a cluster centroid
  bool is_cluster_centroid;
  // if vertex is handing
  bool is_hanging;
  // if point inside frustum
  bool is_inside_frustum;
  // cluster id
  int cluster_id;
  // Sub graph id to keep track while adding to global graph
  int sub_id;
  // Sub graph index
  int sub_graph_id;
  // Neighbours
  // std::vector<Vertex *> neighbours;
};

enum class ExpandGraphStatus {
  kSuccess = 0,
  kErrorKdTree,
  kErrorCollisionEdge,
  kErrorShortEdge,
  kErrorGeofenceViolated,
  kNull
};

struct ExpandGraphReport {
  ExpandGraphReport()
      : status(ExpandGraphStatus::kNull),
        num_vertices_added(0),
        num_edges_added(0),
        vertex_added(NULL) {}
  ExpandGraphStatus status;
  int num_vertices_added;
  int num_edges_added;
  Vertex* vertex_added;
};

class GraphUtils {
 public:
  GraphUtils();

  // Initialize a new graph.
  void reset();

  double distanceBetweenVertices(Vertex* v, Vertex* u);

  // Could change an index to break down the graph into sug-graphs.
  int generateSubgraphIndex();
  // Generate ID for new vertex.
  int generateVertexID();

  void insertVertexKdtree(Vertex* v);

  bool vertexExist(StateVec s);
  bool stateExist(StateVec* s);
  bool edgeExist(int source_id, int target_id);
  // Basic functions on graph including add new vertex and edge.
  void addVertex(Vertex* v);
  void addEdge(Vertex* v, Vertex* u, double weight);
  void removeEdge(Vertex* v, Vertex* u);
  void removeVertex(int id);
  void clearVertex(int id);

  int getNumVertices() { return graph_->getNumVertices(); }
  int getNumEdges() { return graph_->getNumEdges(); }

  // Vertex* getVertex(int id) { return vertices_map_[id]; }

  Vertex* getVertexByState(StateVec s);

  int getIdCount();
  void updateFrontierVertices();
  bool findShortestPaths(ShortestPathsReport& rep);
  bool findShortestPaths(int source_id, ShortestPathsReport& rep);

  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order, std::vector<int>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order, std::vector<Vertex*>& path);
  void getShortestPath(int target_id, const ShortestPathsReport& rep,
                       bool source_to_target_order,
                       std::vector<Eigen::Vector3d>& path);
  // void getShortestPath(int target_id, const ShortestPathsReport& rep,
  //                      bool source_to_target_order,
  //                      std::vector<State>& path);
  double getShortestDistance(int target_id, const ShortestPathsReport& rep);
  int getParentIDFromShortestPath(int target_id,
                                  const ShortestPathsReport& rep);

  // Nearest neigbor lookup.
  bool getNearestVertex(const StateVec* state, Vertex** v_res);
  bool getNearestVertexInRange(const StateVec* state, double range,
                               Vertex** v_res);
  bool getNearestVertices(const StateVec* state, double range,
                          std::vector<Vertex*>* v_res);

  Vertex* getVertexBySubId(int s_id);

  void findLeafVertices(const ShortestPathsReport& rep);
  Vertex* getVertex(int id) { return vertices_map_[id]; }
  int getNumNeighbours(Vertex* v);

  void clearStateHistory();
  void convertGraphToMsg(stage_planner::Graph& graph_msg);
  void convertMsgToGraph(const stage_planner::Graph& graph_msg);
  // Boost graph
  std::shared_ptr<Graph> graph_;
  // Mapping from vertex id to vertex property.
  std::unordered_map<int, Vertex*> vertices_map_;
  // Mapping from state to existance in the graph. // debug
  std::unordered_map<StateVec*, Vertex*> state_map_;
  // id:  <neighbor id, edge cost>
  std::map<int, std::vector<std::pair<int, double>>> edge_map_;

  int subgraph_ind_ = -1;
  int gain_max = 1;
  int gain_min = 0;

  std::vector<StateVec> state_history_;

  std::vector<int> state_ids_;
  std::list<int> frontier_vertices_ids_;
  std::vector<int> leaf_vertices_ids_;

 private:
  // IDs are non-negative integer from 0 (root node)
  // and increased as adding new sub-graph or new vertices.
  // int subgraph_ind_ = -1;
  int id_count_;

  // Kd-tree for nearest neigbor lookup, also keep all vertices.
  kdtree* kd_tree_;

  // Map from local id for Boost Graph Lib to global ID including
  // <sub-graph-id,vertex-id>. This is mainly for debug purpose.
  std::unordered_map<int, std::pair<int, int>> local_id_map_;
};

// GraphUtils::GraphUtils() {
// }
//
// GraphUtils::~GraphUtils() {
// }

#endif
