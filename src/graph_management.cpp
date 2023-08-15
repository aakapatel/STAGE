#include "graph_management.h"

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

GraphManager::GraphManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), 
      nh_private_(nh_private), 
      m_worldFrameId("world"), 
      m_odometry("/hummingbird/ground_truth/odometry"),
      m_currentHeading(0), 
      bounding_box_size_(12), 
      m_local_fov(40),
      m_ufomap("/ufomap_mapping_server_node/map"), 
      m_max_allowed_Z(4),
      obb_radius_(0.5), 
      mapDepth_(1), 
      m_volumetric_gain_threashold(1),
      m_min_dist_bw_nodes(0.3), 
      iterations_local_(2000), 
      iterations_global_(6000),
      max_edge_length_(4)

{
  
  reset();
  // Set variables
  nh_private_.param("frame_id", m_worldFrameId, m_worldFrameId);
  nh_private_.param("odometry", m_odometry, m_odometry);

  nh_private_.param("local_sliding_space", bounding_box_size_, bounding_box_size_);
  nh_private_.param("local_field_of_view", m_local_fov, m_local_fov);
  nh_private_.param("ufomap", m_ufomap, m_ufomap);
  nh_private_.param("safety_height", m_max_allowed_Z, m_max_allowed_Z);
  nh_private_.param("traversability_radius", obb_radius_, obb_radius_);
  nh_private_.param("planning_check_depth", mapDepth_,mapDepth_);
  nh_private_.param("volumetric_gain_threashold", m_volumetric_gain_threashold,
                   m_volumetric_gain_threashold);
  nh_private_.param("graph_density_coefficient", m_min_dist_bw_nodes,
                   m_min_dist_bw_nodes);
  nh_private_.param("maximum_edge_length", max_edge_length_, max_edge_length_);
  nh_private_.param("maximum_iterations_local", iterations_local_, iterations_local_);
  nh_private_.param("maximum_iterations_global", iterations_global_, iterations_global_);

  // Subscribers

  m_poseSubscriber = nh_.subscribe(m_odometry, 1, &GraphManager::odomCallback, this);

  m_ufoMapSubscriber = nh_.subscribe(m_ufomap, 1, &GraphManager::mapCallback, this);

  // publishers  

  m_frontierPub = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_nodes_vis", 10);

  localGraphVisPub = nh_.advertise<visualization_msgs::MarkerArray>("/local_planning_graph", 10);

  // Services 
  m_frontier_client = nh_.serviceClient<stage_planner::frontierRequest>("request_frontier_target");
  
  m_target_query_client = nh_.serviceClient<stage_planner::isFrontier>("query_frontier_target");

  
  m_exploration_path_srv = nh_.advertiseService("next_exploration_path", &GraphManager::nextExplorationPath, this);

};

// Define an ufoMAP object
ufo::map::OccupancyMapColor ufoMAP_(0.2);

void GraphManager::mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg)
{

  if (ufomap_msgs::msgToUfo(msg->map, ufoMAP_))
  {
    // ROS_INFO("MAP RECEIVED");
    map_received = true;
  }
  else
  {
    ROS_WARN("MAP NOT RECEIVED");
  }
}

Eigen::Vector3d GraphManager::quaternion_to_rpy(Eigen::Quaterniond q)
{
  const double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  const double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
  const double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

  return Eigen::Vector3d{roll, pitch, yaw};
}

Eigen::Quaternion<float> GraphManager::Euler2Quaternion(ufo::math::Vector3 v)
{
  float roll = v.x();
  float pitch = v.y();
  float yaw = v.z();

  Eigen::Quaternion<float> q;

  q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  return q;
}

void GraphManager::odomCallback(
    const nav_msgs::Odometry::ConstPtr &pose)
{
    if (firstTime)
    {
        m_prevPose = *pose;
        firstTime = false;
    }
    else
    {
        m_currentPose = *pose;
    }
    m_currentPosition.x() = m_currentPose.pose.pose.position.x;
    m_currentPosition.y() = m_currentPose.pose.pose.position.y;
    m_currentPosition.z() = m_currentPose.pose.pose.position.z;

    Eigen::Quaterniond attitude{pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
                                pose->pose.pose.orientation.y, pose->pose.pose.orientation.z};

    Eigen::Vector3d euler = quaternion_to_rpy(attitude);

    Eigen::Quaterniond Q (Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()));

    quat_ = Q;  

    tf::Quaternion q(
        pose->pose.pose.orientation.x,
        pose->pose.pose.orientation.y,
        pose->pose.pose.orientation.z,
        pose->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    robotState_[0] = pose->pose.pose.position.x;
    robotState_[1] = pose->pose.pose.position.y;
    robotState_[2] = pose->pose.pose.position.z;
    robotState_[3] = yaw * (180 / M_PI);
}

double GraphManager::calculateDistance(ufo::math::Vector3 point1,
                                       ufo::math::Vector3 point2)
{

  double d =
      sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) +
           pow(point1.z() - point2.z(), 2));
  return d;
}

bool GraphManager::isInCollision(
    ufo::map::OccupancyMapColor const &map,
    ufo::geometry::BoundingVar const &bounding_volume, bool occupied_space,
    bool free_space, bool unknown_space, ufo::map::DepthType min_depth)
{

  for (auto it = map.beginLeaves(bounding_volume, occupied_space, free_space,
                                 unknown_space, false, min_depth),
            it_end = map.endLeaves();
       it != it_end; ++it)
  {
    return true;
  }
  return false;
}

ufo::geometry::OBB GraphManager::makeOBB(ufo::math::Vector3 source,
                                         ufo::math::Vector3 goal,
                                         float radius)
{

  ufo::math::Vector3 direction = goal - source;
  ufo::math::Vector3 center = source + (direction / 2.0);
  double distance = direction.norm();
  direction /= distance;
  double yaw = -atan2(direction[1], direction[0]);
  double pitch = -asin(direction[2]);
  double roll = 0;
  ufo::geometry::OBB obb(center,
                         ufo::map::Point3(distance / 2.0, radius, radius),
                         ufo::math::Quaternion(roll, pitch, yaw));

  return obb;
}


double GraphManager::computeVolumetricGain(ufo::math::Vector3 frontier,
                                           ufo::map::OccupancyMapColor const &map)
{

    double r = 1;
    int unknown_cells, free_cells, occupied_cells = 0;
    ufo::geometry::Sphere sphere(frontier, r);

    for (auto it = map.beginLeaves(sphere, true, true, true, false, 3), it_end = map.endLeaves(); it != it_end; ++it)
    {
        if (it.isUnknown()) {
        unknown_cells++;
        }
        if (it.isFree()) {
        free_cells++;
        }
        if (it.isOccupied()) {
        occupied_cells++;
        }
    }
    
    double volumetric_gain = 10 * (log(unknown_cells / (occupied_cells + 1)));

    return volumetric_gain;
}

ufo::math::Vector3
GraphManager::tunePlanningSpace()
{

  highest_x = -std::numeric_limits<float>::max();
  highest_y = -std::numeric_limits<float>::max();
  highest_z = -std::numeric_limits<float>::max();
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();

  ufo::math::Vector3 minPoint(robotState_[0] - 1 * bounding_box_size_,
                              robotState_[1] - 1 * bounding_box_size_,
                              robotState_[2] - 1 * bounding_box_size_);
  ufo::math::Vector3 maxPoint(robotState_[0] + 1 * bounding_box_size_,
                              robotState_[1] + 1 * bounding_box_size_,
                              robotState_[2] + 1 * bounding_box_size_);
  ufo::geometry::AABB aabb(minPoint, maxPoint);

  for (auto it = ufoMAP_.beginLeaves(aabb, false, true, false, false,
                                 mapDepth_);
       it != ufoMAP_.endLeaves(); ++it)
  {
    if (it.getX() > highest_x)
    {
      highest_x = it.getX();
    }
    if (it.getX() < lowest_x)
    {
      lowest_x = it.getX();
    }
    if (it.getY() > highest_y)
    {
      highest_y = it.getY();
    }
    if (it.getY() < lowest_y)
    {
      lowest_y = it.getY();
    }
    if (it.getZ() > highest_z)
    {
      highest_z = it.getZ();
    }
    if (it.getZ() < lowest_z)
    {
      lowest_z = it.getZ();
    }
  }

  ufo::math::Vector3 planning_space_scaler(
      highest_x - lowest_x, highest_y - lowest_y, highest_z - lowest_z);
  return planning_space_scaler;
}

void GraphManager::sampleNodes (std::vector<ufo::math::Vector3>& nodes) {

  nodes.clear();

  nodes.push_back(m_currentPosition);

  srand(time(0));
  int itterations = 0;

  while (itterations < iterations_local_) {
  
    // Generate a random point

    ufo::math::Vector3 box = tunePlanningSpace();

    float x = lowest_x + abs(1024 * rand() / (RAND_MAX + 1.0)) * box.x();
    float y = lowest_y + abs(1024 * rand() / (RAND_MAX + 1.0)) * box.y();
    float z = lowest_z + abs(1024 * rand() / (RAND_MAX + 1.0)) * box.z();
    ufo::math::Vector3 random_point(x, y, z);
    
    ufo::geometry::Sphere point_sphere(random_point, obb_radius_);
    if (!isInCollision(ufoMAP_, point_sphere, true, false, true,
                       mapDepth_) and
        isInCollision(ufoMAP_, point_sphere, false, true, false,
                      mapDepth_)) 
    {
      nodes.push_back(random_point);  
    }
      itterations++;
  }
  
  // ROS_INFO_STREAM ("Sampled : " << nodes.size() << " nodes" << "\n ");

} // sampleNodes

bool GraphManager::isInGraph (State stat) {
  
  
  bool exist_in_graph = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator> vi;
  graph_->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = graph_->getVertexProperty(it);
    if (stat == vertices_map_[id]->state) {
      exist_in_graph = true;
    } else {
      exist_in_graph = false;
    }
  }

  return exist_in_graph;
}

void GraphManager::buildGraph () {

  // GraphManager::reset();

  std::vector<ufo::math::Vector3> nodes;
  GraphManager::sampleNodes(nodes);
  
  for (int i = 0; i < (nodes.size() - 1); i++) {

  State s1, s2;
  s1.x() = nodes[i].x();
  s1.y() = nodes[i].y();
  s1.z() = nodes[i].z();
  
  Vertex* u;

  if (!isInGraph(s1)) {
    u = new Vertex(generateVertexID(), s1);
    GraphManager::addVertex(u);
  }

    for (int i2 = i; i2 < nodes.size(); i2++) {
      
      s2.x() = nodes[i2].x(); 
      s2.y() = nodes[i2].y(); 
      s2.z() = nodes[i2].z();
      
      Vertex* v;
      if (!isInGraph(s2)) {
        v = new Vertex(generateVertexID(), s2);
        GraphManager::addVertex(v);
      }

      double distance_between_nodes = calculateDistance(nodes[i], nodes[i2]);

      if (distance_between_nodes < max_edge_length_) {
        
        ufo::geometry::OBB obb = makeOBB(nodes[i], nodes[i2], obb_radius_);

        if (!isInCollision(ufoMAP_, obb, true, false, true, mapDepth_)) {
          
          GraphManager::addEdge(u, v, distance_between_nodes);

        }
      }
    }
  }

  ROS_INFO_STREAM ("Built graph with " << getNumVertices() << " nodes & " << getNumEdges() << " edges" << "\n"); 

}


// GRAPH UTILS

void GraphManager::reset() {

  // Reset kdtree first.
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);

  // Reset graph.
  graph_.reset(new Graph());

  // Vertex mapping.
  // Clear memory for all vertex pointers
  if (vertices_map_.size() > 0) {
    for (int i = 0; i < vertices_map_.size(); ++i) {
      delete vertices_map_[i];
    }
  }
  vertices_map_.clear();
  edge_map_.clear();

  // Other params.
  subgraph_ind_ = -1;
  id_count_ = -1;
}

double GraphManager::distanceBetweenVertices (Vertex*v, Vertex*u) {
  
  return sqrt(pow(v->state.x() - u->state.x(), 2 ) 
              +pow(v->state.y() - u->state.y(), 2 )
                +pow(v->state.z() - u->state.z(), 2 ));

}

void GraphManager::addVertex(Vertex* v) {
  kd_insert3(kd_tree_, v->state.x(), v->state.y(), v->state.z(), v);
  if (v->id == 0)
    graph_->addSourceVertex(0);
  else
    graph_->addVertex(v->id);
  vertices_map_[v->id] = v;
}

void GraphManager::addEdge(Vertex* v, Vertex* u, double weight) {
  graph_->addEdge(v->id, u->id, weight);
  edge_map_[v->id].push_back(std::make_pair(u->id, weight));
  edge_map_[u->id].push_back(std::make_pair(v->id, weight));
}

void GraphManager::removeEdge(Vertex* v, Vertex* u) {
  graph_->removeEdge(v->id, u->id);
}

// int GraphManager::getNumVertices() { 
//   return graph_->getNumVertices(); 
// }
//
// int GraphManager::getNumEdges() { 
//   return graph_->getNumEdges(); 
// }
//
// Vertex* GraphManager::getVertex(int id) { 
//   return vertices_map_[id]; 
// }

bool GraphManager::getNearestVertex(const State* state, Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  kd_res_free(nearest);
  return true;
}

bool GraphManager::getNearestVertexInRange(const State* state, double range,
                                           Vertex** v_res) {
  if (getNumVertices() <= 0) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *v_res = (Vertex*)kd_res_item_data(nearest);
  Eigen::Vector3d dist;
  dist << state->x() - (*v_res)->state.x(), state->y() - (*v_res)->state.y(),
      state->z() - (*v_res)->state.z();
  kd_res_free(nearest);
  if (dist.norm() > range) return false;
  return true;
}

bool GraphManager::getNearestVertices(const State* state, double range,
                                      std::vector<Vertex*>* v_res) {
  // Notice that this might include the same vertex in the result.
  // if that vertex is added to the tree before.
  // Use the distance 0 or small threshold to filter out.
  kdres* neighbors =
      kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) return false;
  v_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    Vertex* new_neighbor = (Vertex*)kd_res_item_data(neighbors);
    v_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

bool GraphManager::findShortestPaths(ShortestPathsReport& rep) {
  return graph_->findDijkstraShortestPaths(0, rep);
}

bool GraphManager::findShortestPaths(int source_id, ShortestPathsReport& rep) {
  return graph_->findDijkstraShortestPaths(source_id, rep);
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<Vertex*>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.push_back(vertices_map_[*p]);
  }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<Eigen::Vector3d>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.emplace_back(Eigen::Vector3d(vertices_map_[*p]->state.x(),
                                      vertices_map_[*p]->state.y(),
                                      vertices_map_[*p]->state.z()));
  }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<State>& path) {
  std::vector<int> path_id;
  getShortestPath(target_id, rep, source_to_target_order, path_id);
  for (auto p = path_id.begin(); p != path_id.end(); ++p) {
    path.emplace_back(vertices_map_[*p]->state);
  }
}

void GraphManager::getShortestPath(int target_id,
                                   const ShortestPathsReport& rep,
                                   bool source_to_target_order,
                                   std::vector<int>& path) {
  path.clear();
  if (!rep.status) {
    return;
  }

  if (rep.parent_id_map.size() <= target_id) {
    return;
  }

  if (target_id == rep.source_id) {
    path.push_back(target_id);
    return;
  }

  int parent_id = rep.parent_id_map.at(target_id);
  if (parent_id == target_id) {
    return;
  }

  path.push_back(target_id);  // current vertex id first
  path.push_back(
      parent_id);  // its first parent, the rest is recursively looked up
  while (parent_id != rep.source_id) {
    parent_id = rep.parent_id_map.at(parent_id);
    path.push_back(parent_id);
  }

  // Initially, the path follows target to source order. Reverse if required.
  if (source_to_target_order) {
    std::reverse(path.begin(), path.end());
  }
}

double GraphManager::getShortestDistance(int target_id,
                                         const ShortestPathsReport& rep) {
  double dist = std::numeric_limits<double>::max();

  if (!rep.status) {
    return dist;
  }
  if (rep.parent_id_map.size() <= target_id) {
    return dist;
  }
  dist = rep.distance_map.at(target_id);
  return dist;
}

int GraphManager::getParentIDFromShortestPath(int target_id,
                                              const ShortestPathsReport& rep) {
  if (!rep.status) {
    return target_id;
  }

  if (rep.parent_id_map.size() <= target_id) {
    return target_id;
  }

  return rep.parent_id_map.at(target_id);
}



int GraphManager::generateSubgraphIndex() { return ++subgraph_ind_; }

int GraphManager::generateVertexID() { 
  return id_count_++;
}



ufo::math::Vector3 GraphManager::pointToUfo(geometry_msgs::Point p)
{
  ufo::math::Vector3 v(p.x, p.y, p.z);
  return v;
}

std::vector<ufo::math::Vector3> GraphManager::pointsToUfo(std::vector<geometry_msgs::Point> points)
{

  std::vector<ufo::math::Vector3> ufo_vec;

  for (geometry_msgs::Point p : points)
  {
    ufo_vec.push_back(ufo::math::Vector3(p.x, p.y, p.z));
  }

  return ufo_vec;
}

void GraphManager::fetchExplorationTargets()
{
  
  GraphManager::buildGraph ();
  
  GraphManager::visualizeGraph ();

  stage_planner::frontierRequest frontier_srv;

  if (!run_global_planner)
  {
    frontier_srv.request.local = true;
  }
  else
  {
    frontier_srv.request.force_global = true;
  }

  if (m_frontier_client.call(frontier_srv))
  {

    if (frontier_srv.response.local_nav)
    {

      local_exploration = true;
      global_exploration = false;

      m_local_exploration_target = pointToUfo(frontier_srv.response.best_local);
    }
    else if (frontier_srv.response.local_nav == false)
    {

      local_exploration = false;
      global_exploration = true;

      m_global_repos_targets = pointsToUfo(frontier_srv.response.global_targets);
    }
  }
}

bool GraphManager::isLastTargetFrontier () {

  stage_planner::isFrontier frontier_query_srv;
  geometry_msgs::Point last_target_frontier;
  bool is_last_target_frontier;

  last_target_frontier.x = exploration_target.x();
  last_target_frontier.y = exploration_target.y();
  last_target_frontier.z = exploration_target.z();

  frontier_query_srv.request.frontier_point = last_target_frontier;

  if (m_target_query_client.call(frontier_query_srv)) {

    if (frontier_query_srv.response.is_frontier) {
      is_last_target_frontier = true;
    } else {
      is_last_target_frontier = false;
    }
  }
  return is_last_target_frontier;
}

bool GraphManager::nextExplorationPath(stage_planner::explorationPath::Request &req,
                                        stage_planner::explorationPath::Response &res)

{

  fetchExplorationTargets ();

  // TODO Homing path to be included in this ros service 

  if (req.exploration) {
    // res.path = exploration_path_to_publish_;
    if (local_exploration) { 
      res.planning_status_local = true;
      res.planning_status_global = false;
    } else if (global_exploration ) {
      res.planning_status_global = true;
      res.planning_status_local = false;
    }
  } 

  if (req.homing) {
    // res.path = homing_path_;
      res.planning_status_local = false;
      res.planning_status_global = false;
      res.planning_status_homing = true;
  }

  return true;
  
}



void GraphManager::visualizeGraph() {
  std::shared_ptr<Graph> g = graph_;
  std::unordered_map<int, Vertex*>& v_map = vertices_map_;

  if (getNumVertices() == 0) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = "world";
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = "world";
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 125.0 / 255.0;
  vertex_marker.color.g = 42.0 / 255.0;
  vertex_marker.color.b = 104.0 / 255.0;
  vertex_marker.color.a = 1.0;
  // vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  localGraphVisPub.publish(marker_array);

}
/*

TODO:
  
  1. Homing planner to be running in the background -- done 

  2. Rosservice to fetch new exploration path

  3. Convert path to trajectory in stage_interface.

  4. Add initialization behaviour in stage_interface.

  3. Rosservice to fetch return to home path 

  4. Rosnode health monitoring 

*/
