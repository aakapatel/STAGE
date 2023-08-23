#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include "graph.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <numeric>
#include <unordered_map>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <kdtree/kdtree.h>
#include <pcl/common/distances.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

typedef Eigen::Vector4d State; // x, y, z, yaw

struct Vertex {
  Vertex(int v_id, State v_state) {
    id = v_id;
    state << v_state[0], v_state[1], v_state[2], v_state[3];
    parent = NULL;
    distance = 0;
    gain = 0;
    edges = 0;
    is_root = false;
    is_target = false;
    sub_id = 0;
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
  // number of edges 
  int edges;
  // If it is root vertex 
  bool is_root;
  // If vertex is a potential target 
  bool is_target;
  // Sub graph id to keep track while adding to global graph 
  int sub_id;
};



class GraphUtils {
public:
  
  GraphUtils();

  // Initialize a new graph.
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
  void removeVertex(int id);


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
  
  Vertex* getVertexBySubId (int s_id);
  

  // Boost graph 
  std::shared_ptr<Graph> graph_;
  // Mapping from vertex id to vertex property.
  std::unordered_map<int, Vertex*> vertices_map_;
  // id:  <neighbor id, edge cost>
  std::map<int, std::vector<std::pair<int, double>>> edge_map_;  

private:
  
  // IDs are non-negative integer from 0 (root node)
  // and increased as adding new sub-graph or new vertices.
  int subgraph_ind_ = -1;
  int id_count_ = -1;
  
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
