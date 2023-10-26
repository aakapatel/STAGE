#include "graph_management.h"
#include <std_msgs/ColorRGBA.h>

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

GraphManager::GraphManager(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                           MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel> *map_manager)
  : nh_(nh),
  nh_private_(nh_private),
  map_manager_(map_manager),
  m_worldFrameId("world"),
  m_odometry("/hummingbird/ground_truth/odometry"),
  m_currentHeading(0),
  bounding_box_size_(12),
  m_local_fov(40),
  m_max_allowed_Z(4),
  obb_radius_(0.5),
  mapDepth_(1),
  m_volumetric_gain_threashold(1),
  m_min_dist_bw_nodes(0.3),
  iterations_local_(2000),
  iterations_global_(6000),
  max_edge_length_(4),
  num_edges_max_(10),
  knn_range_(4),
  robot_type_("kAerial")

{
  // map_manager_ =
  //    new MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>(
  //        nh_, nh_private_);
  // Set variables
  nh_private_.param("frame_id", m_worldFrameId, m_worldFrameId);
  nh_private_.param("odometry_topic", m_odometry, m_odometry);
  nh_private_.param("local_AABB_size", bounding_box_size_, bounding_box_size_);
  nh_private_.param("local_FOV", m_local_fov, m_local_fov);
  nh_private_.param("max_edge_length", max_edge_length_, max_edge_length_);
  nh_private_.param("num_edges_max", num_edges_max_, num_edges_max_);
  nh_private_.param("sampling_iterations", iterations_local_, iterations_local_);
  nh_private_.param("knn_range", knn_range_, knn_range_);
  nh_private_.param("safety_margin", knn_range_, knn_range_);
  nh_private_.param("robot_size", robot_box_size, robot_box_size);
  nh_private_.param("robot_type", robot_type_, robot_type_);
  nh_private_.param("truncate_sample_Z", truncate_sample_Z_, truncate_sample_Z_);

  // Subscribers

  m_poseSubscriber = nh_.subscribe(m_odometry, 1, &GraphManager::odomCallback, this);

  // publishers

  m_frontierPub = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_nodes_vis", 10);

  local_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_planning_graph", 10);

  global_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/global_planning_graph", 10);

  frontier_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_graph", 10);

  pose_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/pose_graph", 10);

  local_Paths_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_dijkstra_paths", 10);

  global_Paths_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/global_dijkstra_paths", 10);

  improved_global_Paths_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/improved_global_dijkstra_paths", 10);

  reposition_Path_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/global_repos_path", 10);

  overlap_Vis_Pub_ = nh_.advertise<visualization_msgs::Marker>("/overlap_point", 10);

  random_Nodes_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/random_nodes", 10);

  sparse_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/sparse_graph", 10);

  execution_Time_Pub_ = nh_.advertise<std_msgs::Int64>("/stage_execution_time", 1);

  // Services
  m_frontier_client = nh_.serviceClient<stage_planner::frontierRequest>("request_frontier_target");

  m_target_query_client = nh_.serviceClient<stage_planner::isFrontier>("query_frontier_target");

  m_exploration_path_srv = nh_.advertiseService("next_exploration_path", &GraphManager::nextExplorationPath, this);

  graph_timer_ = nh_.createTimer(ros::Duration(1), &GraphManager::updateGraph, this);

  robot_box_size_ << robot_box_size[0], robot_box_size[1], robot_box_size[2];
  safety_margin_ << safety_margin[0], safety_margin[1], safety_margin[2];

  if (robot_type_ == "kLegged")
  {
    truncate_sample_Z_ = 1;
  }

  map_manager_->setRobotRadius(robot_box_size_.norm() / 2.0);
};


Eigen::Vector3d GraphManager::quaternion_to_rpy(Eigen::Quaterniond q)
{
  const double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  const double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
  const double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

  return Eigen::Vector3d{roll, pitch, yaw};
}

Eigen::Quaternion<float> GraphManager::Euler2Quaternion(Eigen::Vector3d v)
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

int GraphManager::assignVertexID()
{
  return ++vertex_id_;
}

void GraphManager::odomCallback(const nav_msgs::Odometry::ConstPtr &pose)
{

  if (firstTime)
    {

      if (pose_graph_ != NULL)
      {
        pose_graph_->reset();
      }
      pose_graph_.reset(new GraphUtils());
      pose_graph_rep_.reset();

      GraphManager::resetKdtree();
      // GraphManager::resetLocalGraph();
      GraphManager::resetGlobalGraph();
      GraphManager::resetFrontierGraph();
      m_prevPose = *pose;
      firstTime = false;
      State home_state;
      home_state[0] = m_prevPose.pose.pose.position.x;
      home_state[1] = m_prevPose.pose.pose.position.y;
      home_state[2] = m_prevPose.pose.pose.position.z;

      Vertex *home_vertex = new Vertex(pose_graph_->generateVertexID(), home_state);
      pose_graph_->addVertex(home_vertex);
    }
  else
  {
    m_currentPose = *pose;

    State newPoint(pose->pose.pose.position.x,
                   pose->pose.pose.position.y,
                   pose->pose.pose.position.z);
    State oldPoint(m_prevPose.pose.pose.position.x,
                   m_prevPose.pose.pose.position.y,
                   m_prevPose.pose.pose.position.z);

    if (calculateDistance(newPoint, oldPoint) > 1)
    {
      m_prevPose = *pose;

      Vertex *vertex = new Vertex(pose_graph_->generateVertexID(), robotState_);
      pose_graph_->addVertex(vertex);
      int id = pose_graph_->getIdCount();
      Vertex *previous_vertex = pose_graph_->vertices_map_[id - 1];
      pose_graph_->addEdge(vertex, previous_vertex, pose_graph_->distanceBetweenVertices(vertex, previous_vertex));
      Vertex *pose_vertex = new Vertex(global_graph_->generateVertexID(), robotState_);
      pose_vertex->is_root = true;
      global_graph_->addVertex(pose_vertex);
      
      global_graph_->state_history_.push_back(robotState_);
    }
  }

  Eigen::Quaterniond attitude{pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
    pose->pose.pose.orientation.y, pose->pose.pose.orientation.z};

  Eigen::Vector3d euler = quaternion_to_rpy(attitude);

  Eigen::Quaterniond Q(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()));

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

  // GraphManager::visualizeFrontierGraph();
  GraphManager::visualizePoseGraph();
}

double GraphManager::calculateDistance(State s1,
                                       State s2)
{

  double d =
    sqrt(pow(s1.x() - s2.x(), 2) + pow(s1.y() - s2.y(), 2) +
         pow(s1.z() - s2.z(), 2));
  return d;
}

double GraphManager::computeExplorationGain(State state)
{

  double step_size = 0.1;
  Eigen::Vector3d check_state(state[0], state[1], state[2]);
  Eigen::Vector3d increment(step_size, step_size, step_size / 2);

  // double occupied, free, unknown = 0;
  MapManager::VoxelStatus vs = map_manager_->getBoxStatus(check_state, increment, true);
  while (MapManager::VoxelStatus::kFree == vs)
  {
    step_size = step_size + 0.1;
    Eigen::Vector3d increment(step_size, step_size, step_size / 2);
    // free++;
    MapManager::VoxelStatus vs = map_manager_->getBoxStatus(check_state, increment, true);
    if (vs == MapManager::VoxelStatus::kUnknown)
      {
        step_size = 4 * step_size;
        break;
      }
    else if (vs == MapManager::VoxelStatus::kOccupied)
    {
    step_size = step_size / 2;
    break;
  }
  }

  return 10*(step_size);
  // return free.size();
}

void GraphManager::sampleNodes(std::vector<node *> &nodes, State state, int iterations_max, double bounding_box_size)
{

  nodes.clear();

  node *current_pos = new node(state.x(), state.y(), state.z());
  current_pos->addParent(nullptr);
  nodes.push_back(current_pos);

  Eigen::Vector3d center (robotState_[0], robotState_[1], robotState_[2]);
  Eigen::Vector3d bb_size (bounding_box_size_, bounding_box_size_, bounding_box_size_);
  std::vector<Eigen::Vector3d> occupied_voxels;
  std::vector<Eigen::Vector3d> free_voxels;
  
  map_manager_->extractLocalMap(center, bb_size, occupied_voxels, free_voxels);

  highest_x = std::numeric_limits<float>::max();
  highest_y = std::numeric_limits<float>::max();
  highest_z = std::numeric_limits<float>::max();
  lowest_x = -std::numeric_limits<float>::max();
  lowest_y = -std::numeric_limits<float>::max();
  lowest_z = -std::numeric_limits<float>::max();

  for (int i = 0; i < free_voxels.size(); ++i)
  {
    if (free_voxels[i].x() < highest_x)
    {
      highest_x = free_voxels[i].x();
    }
    if (free_voxels[i].x() > lowest_x)
    {
      lowest_x = free_voxels[i].x();
    }
    if (free_voxels[i].y() < highest_y)
    {
      highest_y = free_voxels[i].y();
    }
    if (free_voxels[i].y() > lowest_y)
    {
      lowest_y = free_voxels[i].y();
    }
    if (free_voxels[i].z() < highest_z)
    {
      highest_z = free_voxels[i].z();
    }
    if (free_voxels[i].z() > lowest_z)
    {
      lowest_z = free_voxels[i].z();
    }
  }
  srand(time(0));
  int itterations = 0;

  Eigen::Vector3d box (highest_x - lowest_x, highest_y - lowest_y, highest_z - lowest_z);
  
  while (itterations < iterations_max)
  {
    // Generate a random point

    float z;
    float x = lowest_x + abs(1024 * rand() / (RAND_MAX + 1.0)) * box[0];
    float y = lowest_y + abs(1024 * rand() / (RAND_MAX + 1.0)) * box[1];
    
    // float x = box[0] * (float) rand() / (RAND_MAX);
    // float y = box[1] * (float) rand() / (RAND_MAX);

    if (robot_type_ == "kAerial")
      {

        z = lowest_z + abs(1024 * rand() / (RAND_MAX + 1.0)) * box[2];
      }
    else
    {

      z = robotState_[2] + abs(1024 * rand() / (RAND_MAX + 1.0)) * truncate_sample_Z_;
    }
    Eigen::Vector3d random_state(x, y, z);
    if (MapManager::VoxelStatus::kFree ==
      map_manager_->getBoxStatus(random_state + safety_margin_, robot_box_size_, true))
    {

      float distance = std::numeric_limits<float>::max();
      node *parent;
      std::vector<node *>::iterator it_node;
      for (it_node = nodes.begin(); it_node != nodes.end();
      it_node++)
      {
        Eigen::Vector3d direction = random_state - *((*it_node)->point);
        double new_distance = abs(direction.norm());
        if (new_distance < distance)
        {
          distance = new_distance;
          parent = *it_node;
        }
      };
      Eigen::Vector3d start_pos(parent->point->x(), parent->point->y(), parent->point->z());
      if (MapManager::VoxelStatus::kFree ==
        map_manager_->getPathStatus(random_state, start_pos, robot_box_size_, true))
      {
        node *new_node = new node(x, y, z);
        new_node->addParent(parent);
        parent->addChild(new_node);
        nodes.push_back(new_node);
      }
    }
    itterations++;
  }

  // ROS_INFO_STREAM ("Sampled : " << nodes.size() << " nodes" << "\n ");

} // sampleNodes


State GraphManager::generateRandomState(State root, State goal)
{

  State random_state;

  srand(time(0));

  float x = root[0] + abs(1024 * rand() / (RAND_MAX + 1.0)) * goal[0] - root[0];
  float y = root[1] + abs(1024 * rand() / (RAND_MAX + 1.0)) * goal[1] - root[1];
  float z;

  if (robot_type_ == "kAerial")
    {
      z = root[2] + abs(1024 * rand() / (RAND_MAX + 1.0)) * goal[2] - root[2];
    }
  else
  {
    z = robotState_[2] + abs(1024 * rand() / (RAND_MAX + 1.0)) * truncate_sample_Z_;
  }

  random_state[0] = x;
  random_state[1] = y;
  random_state[2] = z;

  return random_state;
}

State GraphManager::eigenToState(Eigen::Vector3d v)
{
  State s;
  s.x() = v.x();
  s.y() = v.y();
  s.z() = v.z();
  return s;
}

void GraphManager::resetLocalGraph()
{

  if (sub_graph_ != NULL)
  {
    sub_graph_->reset();
  }
  sub_graph_.reset(new GraphUtils());
  sub_graph_rep_.reset();
  ++subgraph_ind_;
  sub_graph_->subgraph_ind_ = subgraph_ind_;
}

void GraphManager::resetGlobalGraph()
{

  Vertex *home;

  if (reset_global_graph)
  {
    // Adding home vertex at initialization phase for simpler RTB planning
    State home_state;
    home_state[0] = robotState_[0];
    home_state[1] = robotState_[1];
    home_state[2] = robotState_[2] + 1;
    home = new Vertex(assignVertexID(), home_state);
    GraphManager::insertVertexKdtree(home);
    global_vertices_ids_.push_back(home->id);
    reset_global_graph = false;
  }

  if (global_graph_ != NULL)
  {
    global_graph_->reset();
  }
  global_graph_.reset(new GraphUtils());
  global_graph_rep_.reset();
  // Vertex *home_vertex = new Vertex(global_graph_->generateVertexID(), home->state);
  // home_vertex->is_root = true;
  // home_vertex->is_leaf_vertex = false;
  // global_graph_->addVertex(home_vertex);
}

void GraphManager::resetFrontierGraph()
{

  if (frontier_graph_ != NULL)
  {
    frontier_graph_->reset();
  }
  frontier_graph_.reset(new GraphUtils());
  frontier_graph_rep_.reset();
}

void GraphManager::resetSparseGraph()
{

  if (sparse_graph_ != NULL)
  {
    sparse_graph_->reset();
  }
  sparse_graph_.reset(new GraphUtils());
  sparse_graph_rep_.reset();
}

void GraphManager::insertVertexKdtree(Vertex *v)
{

  all_vertices.push_back(v);
  kd_insert3(kd_tree_, v->state.x(), v->state.y(), v->state.z(), v);
}

void GraphManager::resetKdtree()
{
  all_vertices.clear();

  // BUG can not free memory

  // if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);
}

bool GraphManager::getNeighbours(const State *state, double range, std::vector<Vertex *> *v_res)
{

  // Notice that this might include the same vertex in the result.
  // if that vertex is added to the tree before.
  // Use the distance 0 or small threshold to filter out.
  kdres *neighbors =
    kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0)
    return false;
  v_res->clear();
  for (int i = 0; i < neighbors_size; ++i)
  {
    Vertex *new_neighbor = (Vertex *)kd_res_item_data(neighbors);
    v_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0)
      break;
  }
  kd_res_free(neighbors);
  return true;
}

void GraphManager::updateKdtree()
{
  
  if (global_graph_->getNumVertices() == 0) {
    return;
  }

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;

  std::pair<Graph::GraphType::edge_iterator,
  Graph::GraphType::edge_iterator>
  ei;
  global_graph_->graph_->getVertexIterator(vi);
  global_graph_->graph_->getEdgeIterator(ei);

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = global_graph_->graph_->getVertexProperty(it);
    if (global_graph_->vertices_map_[id]->is_frontier)
    {
      global_graph_->vertices_map_[id]->is_frontier = false;

      std::vector<Vertex *> knn;
      global_graph_->getNearestVertices(&global_graph_->vertices_map_[id]->state, 2, &knn);
      for (Vertex *v : knn)
      {
        v->is_leaf_vertex = false;
      }

    }
  }
}

double GraphManager::getPathLength(std::vector<Vertex *> path)
{

  double l = 0;

  for (int i = 0; i < path.size() - 1; ++i)
  {
    l = l + calculateDistance(path[i]->state, path[i + 1]->state);
  }
  return l;
}

void GraphManager::printVoxelStatus(MapManager::VoxelStatus vs)
{

  // Only for DEBUG

  if (MapManager::VoxelStatus::kUnknown == vs)
  {
    ROS_INFO("Unknown box \n");
  }
  if (MapManager::VoxelStatus::kFree == vs)
  {
    ROS_INFO("Free box \n");
  }
  if (MapManager::VoxelStatus::kOccupied == vs)
  {
    ROS_INFO("Occupied box \n");
  }
}

bool GraphManager::buildLocalGraph()
{

  bool graph_status = false;
  // high_resolution_clock::time_point start_total = high_resolution_clock::now();

  GraphManager::resetLocalGraph();

  latest_sub_graph_id = sub_graph_->subgraph_ind_;

  root_vertex = new Vertex(sub_graph_->generateVertexID(), robotState_);
  root_vertex->is_root = true;
  root_vertex->sub_id = root_vertex->id;
  root_vertex->sub_graph_id = latest_sub_graph_id;
  root_vertex->gain = computeExplorationGain(root_vertex->state);
  sub_graph_->addVertex(root_vertex);
  
  // Insert the sub graph vertedx to global graph to re-use the SG
  global_graph_->addVertex(new Vertex(global_graph_->generateVertexID(), robotState_));
  std::vector<node *> nodes;
  nodes.clear();

  GraphManager::sampleNodes(nodes, robotState_, iterations_local_, bounding_box_size_);
  if (nodes.empty())
  {
    graph_status = false;
    ROS_WARN("Could not sample nodes in traversable space \n");
    return graph_status;
  }

  // Add randomly sampled nodes in local graph

  for (int i = 0; i < nodes.size(); i++)
  {
    Vertex *vertex = new Vertex(sub_graph_->generateVertexID(), eigenToState(*(nodes[i]->point)));
    vertex->is_local = true;
    vertex->is_frontier = false;
    vertex->sub_id = vertex->id;
    vertex->gain = computeExplorationGain(vertex->state);
    vertex->sub_graph_id = latest_sub_graph_id;
    sub_graph_->addVertex(vertex);
    
    // Insert the sub graph vertedx to global graph to re-use the SG
    global_graph_->addVertex(new Vertex(global_graph_->generateVertexID(), eigenToState(*(nodes[i]->point))));
  }

  // Add local exploration targets to graph

  for (int i = 0; i < local_exploration_targets_.size(); ++i) {

    Vertex *temp_vert = new Vertex (i, local_exploration_targets_[i]); 
    temp_vert->is_leaf_vertex = true;
    temp_vert->is_local = true;
    sub_graph_->insertVertexKdtree(temp_vert);
  }
  
  for (int i = 0; i < sub_graph_->getNumVertices(); ++i)
  {

    std::vector<Vertex *> knn;
    sub_graph_->getNearestVertices(&sub_graph_->vertices_map_[i]->state, 1, &knn);

    for (Vertex *v : knn)
    {
      if (v->is_leaf_vertex and v->is_local)
      {
        sub_graph_->vertices_map_[i]->is_frontier = true;
        sub_graph_->vertices_map_[i]->is_local = true;
      }
    }
  }

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  sub_graph_->graph_->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = sub_graph_->graph_->getVertexProperty(it);
    
    // VOXBLOX CHECK

    Eigen::Vector3d v_r(root_vertex->state[0], root_vertex->state[1], root_vertex->state[2]);
    Eigen::Vector3d v(sub_graph_->vertices_map_[id]->state[0],
                      sub_graph_->vertices_map_[id]->state[1],
                      sub_graph_->vertices_map_[id]->state[2]);
    MapManager::VoxelStatus vs = map_manager_->getPathStatus(v, v_r, robot_box_size_, true);

    if (MapManager::VoxelStatus::kFree == vs)
    {
      if (sub_graph_->vertices_map_[id]->edges < num_edges_max_ and sub_graph_->distanceBetweenVertices(sub_graph_->vertices_map_[id], root_vertex) < max_edge_length_)

      {
        sub_graph_->addEdge(sub_graph_->vertices_map_[id], root_vertex,
                            sub_graph_->distanceBetweenVertices(sub_graph_->vertices_map_[id], root_vertex));
        sub_graph_->vertices_map_[id]->edges++;
      }
    }

    // Get nearby vertices from the current vertex and connect the them to form local edges
    std::vector<Vertex *> nearby_vertices;
    sub_graph_->getNearestVertices(&sub_graph_->vertices_map_[id]->state, 2, &nearby_vertices);

    for (int i = 0; i < nearby_vertices.size(); i++)
    {
      if (!nearby_vertices[i]->is_leaf_vertex)
      {
        // VOXBLOX EDGE CHECK

        if (sub_graph_->vertices_map_[id]->edges < num_edges_max_ and sub_graph_->distanceBetweenVertices(sub_graph_->vertices_map_[id], nearby_vertices[i]) < max_edge_length_)
        {

          Eigen::Vector3d v_n(nearby_vertices[i]->state[0], nearby_vertices[i]->state[1], nearby_vertices[i]->state[2]);
          MapManager::VoxelStatus vn = map_manager_->getPathStatus(v, v_n, robot_box_size_, true);

          if (MapManager::VoxelStatus::kFree == vn)
          {

            sub_graph_->addEdge(sub_graph_->vertices_map_[id], nearby_vertices[i],
                                sub_graph_->distanceBetweenVertices(sub_graph_->vertices_map_[id], nearby_vertices[i]));
            sub_graph_->vertices_map_[id]->edges++;
          }
        }
      }
    }
  }

  if (sub_graph_->getNumEdges() != 0)
    {
      graph_vec_.push_back(sub_graph_);
      graph_status = true;
    }
  else
  {
    ROS_WARN_STREAM("Sub graph has " << sub_graph_->getNumVertices() << " nodes & " << sub_graph_->getNumEdges() << " edges\n");
  }

  return graph_status;

} // buildLocalGraph

bool GraphManager::planLocalPath()
{

  bool path_status = true;
  // Fetch shortest path report from root vertex to all vertices in the graph
  if (!sub_graph_->findShortestPaths(root_vertex->id, sub_graph_rep_))
  {
    path_status = false;
    ROS_WARN("Could not find dijkstra shortest paths on sub graph \n");
    return path_status;
  }

  local_paths_.clear();

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  sub_graph_->graph_->getVertexIterator(vi);

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = sub_graph_->graph_->getVertexProperty(it);

    if (sub_graph_->vertices_map_[id]->is_local and sub_graph_->vertices_map_[id]->is_frontier)
    {

      std::vector<Vertex *> dijkstra_path;

      dijkstra_path.clear();

      sub_graph_->getShortestPath(sub_graph_->vertices_map_[id]->id, sub_graph_rep_, true, dijkstra_path);

      if (!dijkstra_path.empty())
      {

        std::pair<std::vector<Vertex *>, double> p = std::make_pair(dijkstra_path, getPathLength(dijkstra_path));

        local_paths_.push_back(p);
      }
    }
  }
  

  GraphManager::visualizeLocalGraph();
  if (!local_paths_.empty())
    {
      GraphManager::visualizeLocalPaths(local_paths_);

      std::vector<Vertex *> path;

      std::vector<std::pair<std::vector<Vertex *>, double>> paths_;

      paths_ = local_paths_;

      double gain, path_length, reward;
      double my_reward = -std::numeric_limits<double>::max();

      for (auto i = 0; i < paths_.size(); i++)
      {

        gain = computeExplorationGain(paths_[i].first.back()->state);

        path_length = paths_[i].second;

        // ROS_INFO_STREAM (" Local Info gain : " << 100 * gain << "\n");
        // reward = (100 * gain) - path_length;
        reward = gain/path_length;
        if (reward > my_reward)
          {
            my_reward = reward;
            path = paths_[i].first;
          }
      }

      // best_path_.clear();
      GraphManager::improvePath(path, best_path_);
      if (best_path_.empty())
      {
        ROS_WARN("Improved local path is empty \n");
        path_status = false;
        return path_status;
      }
    }
  else
  {
    ROS_WARN("SG : Could not find shortest paths to frontier vertices \n");
    path_status = false;
    return path_status;
  }

  return path_status;

} // planLocalPath

bool GraphManager::buildSparseGraph () 
{

  // Iterate through global graph, look for root vertices and their neighbours. Add them to sparse graph wwhile updating their frontier/global property.
  
  // The sparse graph should reset every iteration to plan paths in the background with latest map update 
  GraphManager::resetSparseGraph();

  bool graph_status = true;
  
  for (int i = 0; i < global_graph_->getNumVertices(); ++i)
  {

    std::vector<Vertex *> knn;
    if (global_graph_->vertices_map_[i]->is_root) {
      State state = global_graph_->vertices_map_[i]->state; 
      Vertex* sparse_graph_vertex = new Vertex (sparse_graph_->generateVertexID(), state);
      sparse_graph_vertex->is_root = true;
      sparse_graph_->addVertex(sparse_graph_vertex);
      global_graph_->getNearestVertices(&state, knn_range_, &knn);

      for (Vertex *v : knn)
      {
        State state_nn = v->state;
        Vertex * sparse_graph_vertex_nn = new Vertex(sparse_graph_->generateVertexID(),state_nn);
        
        if (v->is_leaf_vertex and v->is_global)
        {
          sparse_graph_vertex_nn->is_frontier = true;
          sparse_graph_vertex_nn->is_global = true;
        }
        sparse_graph_->addVertex(sparse_graph_vertex_nn);
      }
    }
  }

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  
  sparse_graph_->graph_->getVertexIterator(vi);

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    
    std::vector<Vertex *> nearby_vertices;
    int id = sparse_graph_->graph_->getVertexProperty(it);
    Vertex* sparse_graph_vertex = sparse_graph_->vertices_map_[id];
    sparse_graph_->getNearestVertices(&sparse_graph_vertex->state, 2, &nearby_vertices);

    const Eigen::Vector3d v(sparse_graph_vertex->state[0],
                            sparse_graph_vertex->state[1],
                            sparse_graph_vertex->state[2]);

    for (int i = 0; i < nearby_vertices.size(); ++i)
    {

      if (!nearby_vertices[i]->is_leaf_vertex)
      {

        int nearby_vertex_id = nearby_vertices[i]->id;

        const Eigen::Vector3d v_n(sparse_graph_->vertices_map_[nearby_vertex_id]->state[0],
                                  sparse_graph_->vertices_map_[nearby_vertex_id]->state[1],
                                  sparse_graph_->vertices_map_[nearby_vertex_id]->state[2]);

        MapManager::VoxelStatus vs = map_manager_->getPathStatus(v, v_n, robot_box_size_, true);

        int e = sparse_graph_->vertices_map_[id]->edges;
        double weight = sparse_graph_->distanceBetweenVertices(sparse_graph_->vertices_map_[id],
                                                                     sparse_graph_->vertices_map_[nearby_vertex_id]);
        
        if (e < num_edges_max_ && weight < max_edge_length_ && MapManager::VoxelStatus::kFree == vs)
        {
          sparse_graph_->addEdge(sparse_graph_->vertices_map_[id],
                                 sparse_graph_->vertices_map_[nearby_vertex_id],
                                 sparse_graph_->distanceBetweenVertices(sparse_graph_->vertices_map_[id],
                                                                       sparse_graph_->vertices_map_[nearby_vertex_id]));
          
          sparse_graph_->vertices_map_[id]->edges++;
        }
      }
    }
  }

  GraphManager::visualizeSparseGraph();

}

bool GraphManager::buildGlobalGraph()
{
  bool graph_status = true;
  if (global_graph_->getNumVertices() == 0)
  {
    ROS_INFO("Sub graph nodes not inserted into Global graph \n");
    graph_status = false;
    return graph_status;
  } else {
    GraphManager::updateKdtree();
  }

  // Iterate through frontiers and mark leaf vertices 
  
  for (int i = 0; i < global_repos_targets_.size(); ++i) {

    Vertex *temp_vert = new Vertex (++temp_id, global_repos_targets_[i]); 
    temp_vert->is_leaf_vertex = true;
    temp_vert->is_global = true;
    global_graph_->insertVertexKdtree(temp_vert);
  }

  
  // Iterate through global graph leaf vertices to detect potential frontier nodes

  for (int i = 0; i < global_graph_->getNumVertices(); ++i)
  {

    std::vector<Vertex *> knn;
    global_graph_->getNearestVertices(&global_graph_->vertices_map_[i]->state, 1, &knn);

    for (Vertex *v : knn)
    {
      if (v->is_leaf_vertex and v->is_global)
      {
        global_graph_->vertices_map_[i]->is_frontier = true;
        global_graph_->vertices_map_[i]->is_global = true;
      }
    }
  }
  

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  
  global_graph_->graph_->getVertexIterator(vi);

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    
    std::vector<Vertex *> nearby_vertices;
    int id = global_graph_->graph_->getVertexProperty(it);
    Vertex* global_graph_vertex = global_graph_->vertices_map_[id];
    global_graph_->getNearestVertices(&global_graph_vertex->state, 1, &nearby_vertices);

    const Eigen::Vector3d v(global_graph_vertex->state[0],
                            global_graph_vertex->state[1],
                            global_graph_vertex->state[2]);

    for (int i = 0; i < nearby_vertices.size(); ++i)
    {

      if (!nearby_vertices[i]->is_leaf_vertex)
      {

        int nearby_vertex_id = nearby_vertices[i]->id;

        const Eigen::Vector3d v_n(global_graph_->vertices_map_[nearby_vertex_id]->state[0],
                                  global_graph_->vertices_map_[nearby_vertex_id]->state[1],
                                  global_graph_->vertices_map_[nearby_vertex_id]->state[2]);

        MapManager::VoxelStatus vs = map_manager_->getPathStatus(v, v_n, robot_box_size_, true);

        int e = global_graph_->vertices_map_[id]->edges;
        double weight = global_graph_->distanceBetweenVertices(global_graph_->vertices_map_[id],
                                                                     global_graph_->vertices_map_[nearby_vertex_id]);
        
        if (e < num_edges_max_ && weight < max_edge_length_ && MapManager::VoxelStatus::kFree == vs)
        {
          global_graph_->addEdge(global_graph_->vertices_map_[id],
                                 global_graph_->vertices_map_[nearby_vertex_id],
                                 global_graph_->distanceBetweenVertices(global_graph_->vertices_map_[id],
                                                                        global_graph_->vertices_map_[nearby_vertex_id]));
          
          global_graph_->vertices_map_[id]->edges++;
        }
      }
    }
  }

  GraphManager::visualizeGlobalGraph();
  if (global_graph_->getNumEdges() > 2)
    {
      graph_status = true;
    }
  else
  {
    graph_status = false;
    ROS_WARN_STREAM("Global graph status : " << global_graph_->getNumVertices() << " vertices & " << global_graph_->getNumEdges() << " edges\n");
  }

  return graph_status;

} // buildGlobalGraph

bool GraphManager::planGlobalPath()
{

  bool path_status = true;

  // Dijkstra SP report from root to all vertices in global graph
  Vertex* global_root_vertex;
  global_graph_->getNearestVertex(&robotState_, &global_root_vertex);
  global_graph_rep_.reset();
  if (!global_graph_->findShortestPaths(global_root_vertex->id, global_graph_rep_))
  {
    ROS_WARN("Could not find dijkstra shortest paths on global graph \n");
    path_status = false;
    return path_status;
  }

  global_paths_.clear();

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  global_graph_->graph_->getVertexIterator(vi);

  std::vector<std::pair<std::vector<Vertex *>, double>> temp_global_paths;

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = global_graph_->graph_->getVertexProperty(it);

    if (global_graph_->vertices_map_[id]->is_frontier)
    {

      std::vector<Vertex *> dijkstra_path;

      dijkstra_path.clear();

      global_graph_->getShortestPath(global_graph_->vertices_map_[id]->id, global_graph_rep_, true, dijkstra_path);

      if (!dijkstra_path.empty())
      {
        std::pair<std::vector<Vertex *>, double> p = std::make_pair(dijkstra_path, getPathLength(dijkstra_path));

        temp_global_paths.push_back(p);
      }
    }
  }

  GraphManager::visualizeGlobalPaths(temp_global_paths);

  for (int i = 0; i < temp_global_paths.size(); ++i)
  {

    std::vector<Vertex *> improved_path_;
    GraphManager::improvePath(temp_global_paths[i].first, improved_path_);
    if (!improved_path_.empty())
    {
      std::pair<std::vector<Vertex *>, double> p = std::make_pair(improved_path_, getPathLength(improved_path_));
      global_paths_.push_back(p);
    }
  }

  if (!global_paths_.empty())
    {
      GraphManager::visualizeImprovedGlobalPaths(global_paths_);

      std::vector<Vertex *> path;

      std::vector<std::pair<std::vector<Vertex *>, double>> paths_;

      paths_ = global_paths_;

      double gain, path_length, reward;
      double my_reward = -std::numeric_limits<double>::max();

      for (auto i = 0; i < paths_.size(); i++)
      {

        gain = computeExplorationGain(paths_[i].first.back()->state);
        path_length = paths_[i].second;

        // ROS_INFO_STREAM ("Global Info gain : " << 100 * gain << "\n");
        // reward = (100 * gain) - 50 * path_length;
        reward = gain/path_length;
        if (reward > my_reward)
        {
          my_reward = reward;
          path = paths_[i].first;
        }
      }

      // best_path_.clear();
      GraphManager::improvePath(path, best_path_);
      GraphManager::updateKdtree();
      if (best_path_.empty())
      {
        path_status = false;
        ROS_WARN("GG : Improved path is empty");
        return path_status;
      }
    }
  else
  {
    path_status = false;
    ROS_WARN("GG : No shortest paths to frontiers \n");
    return path_status;
  }

  return path_status;

} // planGlobalPaths

bool GraphManager::setPath()
{

  bool path_set = true;
  if (best_path_.empty()) {
    path_set = false;
    return path_set;
  } else {

    GraphManager::visualizeRepositionPath();
    std::vector<Vertex *> path_seg;
    path_seg.clear();
    Eigen::Vector3d v_root;
    if (path_iterator_ < best_path_.size())
      {

        if (path_iterator_ == 0)
          {
            v_root = Eigen::Vector3d(robotState_[0], robotState_[1], robotState_[2]);
          }
        else
        {
          v_root = Eigen::Vector3d(best_path_[path_iterator_ - 1]->state[0], best_path_[path_iterator_ - 1]->state[1], best_path_[path_iterator_ - 1]->state[2]);
        }

        Eigen::Vector3d v_vert(best_path_[path_iterator_]->state[0], best_path_[path_iterator_]->state[1], best_path_[path_iterator_]->state[2]);
        MapManager::VoxelStatus vs = map_manager_->getPathStatus(v_root, v_vert, robot_box_size_, true);

        if (MapManager::VoxelStatus::kFree == vs or MapManager::VoxelStatus::kUnknown == vs)
          {
            State s = Eigen::Vector3d(v_root[0], v_root[1], v_root[2]);

            Vertex *root_vertex = new Vertex(path_iterator_, s);
            path_seg.push_back(root_vertex);
            path_seg.push_back(best_path_[path_iterator_]);
            path_iterator_ = path_iterator_ + 1;
            reached_end_of_path_ = false;

            exploration_path_.poses.clear();
            geometry_msgs::PoseStamped p;

            nav_msgs::Path new_path;
            new_path.poses.clear();
            GraphManager::segmentPath(path_seg, new_path);

            for (int i = 0; i < new_path.poses.size() - 1; i++)
            {

              float angle = atan2(new_path.poses[i + 1].pose.position.y - new_path.poses[i].pose.position.y,
                                  new_path.poses[i + 1].pose.position.x - new_path.poses[i].pose.position.x);
              Eigen::Vector3d v(0, 0, angle);
              Eigen::Quaternion<float> q = Euler2Quaternion(v);

              p.pose.position.x = new_path.poses[i].pose.position.x;
              p.pose.position.y = new_path.poses[i].pose.position.y;
              p.pose.position.z = new_path.poses[i].pose.position.z;
              p.pose.orientation.x = q.x();
              p.pose.orientation.y = q.y();
              p.pose.orientation.z = q.z();
              p.pose.orientation.w = q.w();
              exploration_path_.poses.push_back(p);
            }

          }
        else
        {
          ROS_WARN_STREAM("Path segment in UNTRAVERSABLE zone \n");
          ROS_INFO_STREAM("Terminating path here .. \n");
          exploration_path_.poses.clear();
          reached_end_of_path_ = true;
          path_iterator_ = 0;
          best_path_.clear();
          path_set =  false;
          return path_set;
        }
      }
    else
    {
      ROS_INFO_STREAM("Reached end of path \n");
      exploration_path_.poses.clear();
      reached_end_of_path_ = true;
      path_iterator_ = 0;
      best_path_.clear();
      path_set =  false;
      return path_set;
    }
  }

  return path_set;

}
void GraphManager::improvePath(std::vector<Vertex *> raw_path, std::vector<Vertex *> &path)
{

  // Reset the improvement graph & clear memory

  improvement_graph_.reset(new GraphUtils());
  if (improvement_graph_ != NULL)
  {
    improvement_graph_->reset();
  }
  // Reset SP report
  improvement_graph_rep_.reset();

  // Include root vertex as current robot state
  Vertex *root_vertex = new Vertex(improvement_graph_->generateVertexID(), robotState_);
  improvement_graph_->addVertex(root_vertex);
  root_vertex->is_root = true;

  // Include goal vertex as last vertex of Dijkstra path

  Vertex *goal_vertex = new Vertex(improvement_graph_->generateVertexID(), raw_path.back()->state);
  improvement_graph_->addVertex(goal_vertex);

  // Not ideal, but check nearest neighbours to add edges in a graph that will improve the path
  // by shortcutting some edges

  for (std::vector<Vertex *>::iterator it_vert = raw_path.begin(); it_vert != std::prev(raw_path.end(), 1); it_vert++)
  {

    Vertex *imp_vertex = new Vertex(improvement_graph_->generateVertexID(), (*it_vert)->state);
    improvement_graph_->addVertex(imp_vertex);
  }

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  improvement_graph_->graph_->getVertexIterator(vi);

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = improvement_graph_->graph_->getVertexProperty(it);

    // Get nearby vertices from the current vertex and connect the them to form local edges
    std::vector<Vertex *> nearby_vertices;
    improvement_graph_->getNearestVertices(&improvement_graph_->vertices_map_[id]->state, 10, &nearby_vertices);

    for (int i = 0; i < nearby_vertices.size(); i++)
    {

      Eigen::Vector3d v1(improvement_graph_->vertices_map_[id]->state[0],
                         improvement_graph_->vertices_map_[id]->state[1],
                         improvement_graph_->vertices_map_[id]->state[2]);

      Eigen::Vector3d v2(nearby_vertices[i]->state[0], nearby_vertices[i]->state[1], nearby_vertices[i]->state[2]);
      MapManager::VoxelStatus vn = map_manager_->getPathStatus(v1, v2, robot_box_size_, true);

      double edge_length = improvement_graph_->distanceBetweenVertices(improvement_graph_->vertices_map_[id], nearby_vertices[i]);

      if (edge_length < 3 * max_edge_length_ and MapManager::VoxelStatus::kFree == vn)
      {

        improvement_graph_->addEdge(improvement_graph_->vertices_map_[id], nearby_vertices[i],
                                    improvement_graph_->distanceBetweenVertices(improvement_graph_->vertices_map_[id], nearby_vertices[i]));
      }
    }
  }

  // Dijkstra SP report from root to all vertices in improvement graph

  if (improvement_graph_->findShortestPaths(root_vertex->id, improvement_graph_rep_))
    {
      // ROS_INFO ("Improvement graph dijkstra report : PASS \n");
    }
  else
  {
    ROS_WARN("Could not find dijkstra shortest paths on graph improvement \n");
  }

  improvement_graph_->getShortestPath(goal_vertex->id, improvement_graph_rep_, true, path);
}

void GraphManager::updateGraph(const ros::TimerEvent &event)
{

  // Can off load some repetitive stuff here
}

std::vector<Eigen::Vector3d> GraphManager::pointsToEigen(std::vector<geometry_msgs::Point> points)
{

  std::vector<Eigen::Vector3d> vec;

  for (geometry_msgs::Point p : points)
  {
    Eigen::Vector3d v(p.x, p.y, p.z);
    vec.push_back(v);
  }

  return vec;
}

void GraphManager::fetchExplorationTargets()
{

  stage_planner::frontierRequest frontier_srv;

  global_repos_targets_.clear();
  local_exploration_targets_.clear();

  frontier_srv.request.explore = true;

  if (m_frontier_client.call(frontier_srv))
  {

    if (frontier_srv.response.frontiers_exist)
    {

      // get a frontier target in local FOV
      local_exploration_targets_ = pointsToEigen(frontier_srv.response.local_targets);

      global_repos_targets_ = pointsToEigen(frontier_srv.response.global_targets);

      GraphManager::resetFrontierGraph();

      if (global_repos_targets_.empty() and local_exploration_targets_.empty())
      {
        return;
      }
      std::vector<double> gains;
      gains.clear();
      if (!local_exploration_targets_.empty())
      {

        for (auto i : local_exploration_targets_)
        {
          Vertex *frontier_vertex = new Vertex(frontier_graph_->generateVertexID(), eigenToState(i));
          frontier_vertex->is_local = true;
          frontier_vertex->gain = 10 * computeExplorationGain(eigenToState(i));
          frontier_graph_->addVertex(frontier_vertex);
          gains.push_back(frontier_vertex->gain);
        }
      }

      if (!global_repos_targets_.empty())
      {

        for (auto i : global_repos_targets_)
        {
          Vertex *frontier_vertex = new Vertex(frontier_graph_->generateVertexID(), eigenToState(i));
          frontier_vertex->is_global = true;
          frontier_vertex->gain = 10 * computeExplorationGain(eigenToState(i));
          // ROS_INFO_STREAM ("Frontier vertex gain : " << frontier_vertex->gain << "\n");
          frontier_graph_->addVertex(frontier_vertex);
          gains.push_back(frontier_vertex->gain);
        }
      }

      average_gain = std::accumulate(gains.begin(), gains.end(), 0.0) / gains.size();

      ROS_INFO_STREAM ("Fetched " << local_exploration_targets_.size() << " Local & "
      << global_repos_targets_.size() << " Global frontiers \n");
    }
  }
}

bool GraphManager::isLastTargetFrontier(geometry_msgs::Point last_target_frontier)
{

  stage_planner::isFrontier frontier_query_srv;
  bool is_last_target_frontier;

  last_target_frontier.x = exploration_target.x();
  last_target_frontier.y = exploration_target.y();
  last_target_frontier.z = exploration_target.z();

  frontier_query_srv.request.frontier_point = last_target_frontier;

  if (m_target_query_client.call(frontier_query_srv))
  {

    if (frontier_query_srv.response.is_frontier)
      {
        is_last_target_frontier = true;
      }
    else
    {
      is_last_target_frontier = false;
    }
  }
  return is_last_target_frontier;
}


bool GraphManager::nextExplorationPath(stage_planner::explorationPath::Request &req,
                                       stage_planner::explorationPath::Response &res)

{

  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  
  bool return_value = true;
  
  if (reached_end_of_path_) {
    
    exploration_path_.poses.clear();
    ++planning_iteration_;

    GraphManager::fetchExplorationTargets();
    if (!local_exploration_targets_.empty()) {

      // ROS_INFO ("Local exploration targets received \n ");

      if (GraphManager::buildLocalGraph()) {
        // ROS_INFO ("Successfully built local graph \n");

        if (GraphManager::planLocalPath()) {
          // ROS_INFO ("Successfully planned local path \n");

          if (GraphManager::setPath()) {
            ROS_INFO ("Set path success ! \n");

              if (GraphManager::buildGlobalGraph()) {
                // ROS_INFO ("Successfully built reserved graph \n");
              GraphManager::buildSparseGraph();  
            }

            res.path = exploration_path_;
            high_resolution_clock::time_point stop_total = high_resolution_clock::now();

            auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);

            ROS_INFO_STREAM("\033[;34m Total Execution time \033[0m "
                            << duration_total.count() << " \033[;34m ms \033[0m\n ");
            return true;
          } else {

            // Check if global re-positioning is possible 

            if (!global_repos_targets_.empty()) {
              GraphManager::buildLocalGraph();
              GraphManager::buildGlobalGraph();
              GraphManager::buildSparseGraph();  
              if (GraphManager::planGlobalPath()) {
                ROS_INFO ("Global planner engaged \n");

                if (GraphManager::setPath()) {
                  ROS_INFO ("Successfully planned global re-pos path \n");
                  res.path = exploration_path_;

                  high_resolution_clock::time_point stop_total = high_resolution_clock::now();

                  auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);

                  ROS_INFO_STREAM("\033[;34m Total Execution time \033[0m "
                                  << duration_total.count() << " \033[;34m ms \033[0m\n ");

                  return true;
                } else {
                  ROS_WARN ("Global planner failure !! \n");
                  high_resolution_clock::time_point stop_total = high_resolution_clock::now();

                  auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);

                  ROS_INFO_STREAM("\033[;34m Total Execution time \033[0m "
                                  << duration_total.count() << " \033[;34m ms \033[0m\n ");                     
                }
              }
            }
          }
        }
      }
    } else if (!global_repos_targets_.empty() and local_exploration_targets_.empty()) {

      GraphManager::buildLocalGraph();
      GraphManager::buildGlobalGraph();
      GraphManager::buildSparseGraph();  
      if (GraphManager::planGlobalPath()) {
        ROS_INFO ("Global planner engaged \n");

        if (GraphManager::setPath()) {
          ROS_INFO ("Successfully planned global re-pos path \n");
          res.path = exploration_path_;
          high_resolution_clock::time_point stop_total = high_resolution_clock::now();

          auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);

          ROS_INFO_STREAM("\033[;34m Total Execution time \033[0m "
                          << duration_total.count() << " \033[;34m ms \033[0m\n ");
          return true;
        } else {
          ROS_WARN ("Global planner failure !! \n");
          high_resolution_clock::time_point stop_total = high_resolution_clock::now();

          auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);

          ROS_INFO_STREAM("\033[;34m Total Execution time \033[0m "
                          << duration_total.count() << " \033[;34m ms \033[0m\n ");
          // return false;
        }
      }
    }

  } else {

    return_value = GraphManager::setPath (); 
    
      if (!reached_end_of_path_)
      {
        res.path = exploration_path_;
        ROS_INFO_STREAM("Following path segment \n");
      } else  {
        ROS_INFO ("Reached end.. fetch new path \n");
        }
      high_resolution_clock::time_point stop_total = high_resolution_clock::now();

      auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);

      ROS_INFO_STREAM("\033[;34m Total Execution time \033[0m "
                      << duration_total.count() << " \033[;34m ms \033[0m\n ");      
    
  }

  return return_value;

}

// Visualization of navigation graphs

void GraphManager::visualizeFrontierGraph()
{

  if (all_vertices.empty())
    return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = "world";
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 125.0 / 255.0;
  vertex_marker.color.g = 42.0 / 255.0;
  vertex_marker.color.b = 104.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.frame_locked = false;

  for (auto it = all_vertices.begin(); it != all_vertices.end(); ++it)
  {

    if ((*it)->is_frontier)
    {

      geometry_msgs::Point p1;
      p1.x = (*it)->state[0];
      p1.y = (*it)->state[1];
      p1.z = (*it)->state[2];

      vertex_marker.points.push_back(p1);
    }
  }
  marker_array.markers.push_back(vertex_marker);

  frontier_Graph_Vis_Pub_.publish(marker_array);
}

void GraphManager::visualizeLocalGraph()
{
  std::shared_ptr<Graph> g = sub_graph_->graph_;
  std::unordered_map<int, Vertex *> &v_map = sub_graph_->vertices_map_;

  if (sub_graph_->getNumVertices() == 0)
    return;

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
  edge_marker.color.r = 150.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 0.8;
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
  ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
  {
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
  vertex_marker.scale.x = 0.25;
  vertex_marker.scale.y = 0.25;
  vertex_marker.scale.z = 0.25;
  // vertex_marker.color.r = 125.0 / 255.0;
  // vertex_marker.color.g = 42.0 / 255.0;
  // vertex_marker.color.b = 104.0 / 255.0;
  // vertex_marker.color.a = 1.0;
  vertex_marker.frame_locked = false;

  visualization_msgs::Marker frontier_vertex_marker;
  frontier_vertex_marker.header.stamp = ros::Time::now();
  frontier_vertex_marker.header.seq = 0;
  frontier_vertex_marker.header.frame_id = "world";
  frontier_vertex_marker.id = 0;
  frontier_vertex_marker.ns = "frontier";
  frontier_vertex_marker.action = visualization_msgs::Marker::ADD;
  frontier_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  frontier_vertex_marker.scale.x = 0.4;
  frontier_vertex_marker.scale.y = 0.4;
  frontier_vertex_marker.scale.z = 0.4;
  frontier_vertex_marker.color.r = 255.0 / 255.0;
  frontier_vertex_marker.color.g = 0.0 / 255.0;
  frontier_vertex_marker.color.b = 0.0 / 255.0;
  frontier_vertex_marker.color.a = 1.0;
  frontier_vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  g->getVertexIterator(vi);

  double minZ = std::numeric_limits<double>::max();
  double maxZ = -std::numeric_limits<double>::max();

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = g->getVertexProperty(it);
    if (v_map[id]->gain > maxZ)
    // if (v_map[id]->state[2] > maxZ)
    {
      // maxZ = v_map[id]->state[2];
      maxZ = v_map[id]->gain;
    }

    if (v_map[id]->gain < minZ)
    // if (v_map[id]->state[2] < minZ)
    {
      // minZ = v_map[id]->state[2];
      minZ = v_map[id]->gain;
    }
  }

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    double gain = v_map[id]->gain;
    // double h = (1.0 - std::min(std::max((p1.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * 0.8;
    double h = (1.0 - std::min(std::max((gain - minZ) / (maxZ - minZ), 0.0), 1.0)) * 0.8;
    vertex_marker.colors.push_back(GraphManager::heightMapColor(h));
    vertex_marker.points.push_back(p1);
    if (v_map[id]->is_frontier)
    {

      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];

      frontier_vertex_marker.points.push_back(p1);
    }
  }
  marker_array.markers.push_back(vertex_marker);
  marker_array.markers.push_back(frontier_vertex_marker);
  local_Graph_Vis_Pub_.publish(marker_array);
}

void GraphManager::visualizeLocalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> local_paths)
{

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  visualization_msgs::Marker vertex_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = "world";
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 255.0 / 255.0;
  edge_marker.color.g = 0.0 / 255.0;
  edge_marker.color.b = 255.0 / 255.0;
  edge_marker.color.a = 1.0;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

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
  vertex_marker.color.r = 255.0 / 255.0;
  vertex_marker.color.g = 0.0 / 255.0;
  vertex_marker.color.b = 255.0 / 255.0;
  vertex_marker.color.a = 1.0;
  // vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < local_paths.size() - 1; ++i)
  {

    for (int it = 0; it < local_paths[i].first.size() - 1; ++it)
    {

      geometry_msgs::Point p1;
      geometry_msgs::Point p2;
      p1.x = local_paths[i].first[it]->state[0];
      p1.y = local_paths[i].first[it]->state[1];
      p1.z = local_paths[i].first[it]->state[2];
      p2.x = local_paths[i].first[it + 1]->state[0];
      p2.y = local_paths[i].first[it + 1]->state[1];
      p2.z = local_paths[i].first[it + 1]->state[2];

      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
      vertex_marker.points.push_back(p1);
      vertex_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(edge_marker);
  marker_array.markers.push_back(vertex_marker);

  local_Paths_Vis_Pub_.publish(marker_array);
}

void GraphManager::visualizeGlobalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> global_paths)
{

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  visualization_msgs::Marker vertex_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = "world";
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.1;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 0 / 255.0;
  edge_marker.color.b = 0.0 / 255.0;
  edge_marker.color.a = 1.0;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = "world";
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.5;
  vertex_marker.scale.y = 0.5;
  vertex_marker.scale.z = 0.5;
  vertex_marker.color.r = 200.0 / 255.0;
  vertex_marker.color.g = 0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  // vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < global_paths.size() - 1; ++i)
  {

    for (int it = 0; it < global_paths[i].first.size() - 1; ++it)
    {

      geometry_msgs::Point p1;
      geometry_msgs::Point p2;
      p1.x = global_paths[i].first[it]->state[0];
      p1.y = global_paths[i].first[it]->state[1];
      p1.z = global_paths[i].first[it]->state[2];
      p2.x = global_paths[i].first[it + 1]->state[0];
      p2.y = global_paths[i].first[it + 1]->state[1];
      p2.z = global_paths[i].first[it + 1]->state[2];

      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
      vertex_marker.points.push_back(p1);
      vertex_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(edge_marker);
  marker_array.markers.push_back(vertex_marker);

  global_Paths_Vis_Pub_.publish(marker_array);
}

void GraphManager::visualizeImprovedGlobalPaths(std::vector<std::pair<std::vector<Vertex *>, double>> global_paths)
{

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  visualization_msgs::Marker vertex_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = "world";
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.12;
  edge_marker.color.r = 0.0 / 255.0;
  edge_marker.color.g = 200.0 / 255.0;
  edge_marker.color.b = 0.0 / 255.0;
  edge_marker.color.a = 1.0;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = "world";
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.55;
  vertex_marker.scale.y = 0.55;
  vertex_marker.scale.z = 0.55;
  vertex_marker.color.r = 0.0 / 255.0;
  vertex_marker.color.g = 200.0 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 1.0;
  // vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < global_paths.size() - 1; ++i)
  {

    for (int it = 0; it < global_paths[i].first.size() - 1; ++it)
    {

      geometry_msgs::Point p1;
      geometry_msgs::Point p2;
      p1.x = global_paths[i].first[it]->state[0];
      p1.y = global_paths[i].first[it]->state[1];
      p1.z = global_paths[i].first[it]->state[2];
      p2.x = global_paths[i].first[it + 1]->state[0];
      p2.y = global_paths[i].first[it + 1]->state[1];
      p2.z = global_paths[i].first[it + 1]->state[2];

      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
      vertex_marker.points.push_back(p1);
      vertex_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(edge_marker);
  marker_array.markers.push_back(vertex_marker);

  improved_global_Paths_Vis_Pub_.publish(marker_array);
}

void GraphManager::visualizeRepositionPath()
{

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  visualization_msgs::Marker vertex_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = "world";
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.3;
  edge_marker.color.r = 50.0 / 255.0;
  edge_marker.color.g = 130.0 / 255.0;
  edge_marker.color.b = 150.0 / 255.0;
  edge_marker.color.a = 1.0;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = "world";
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.5;
  vertex_marker.scale.y = 0.5;
  vertex_marker.scale.z = 0.5;
  vertex_marker.color.r = 50.0 / 255.0;
  vertex_marker.color.g = 150.0 / 255.0;
  vertex_marker.color.b = 130.0 / 255.0;
  vertex_marker.color.a = 1.0;
  // vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < best_path_.size() - 1; ++i)
  {
    int it = i + 1;

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    p1.x = best_path_[i]->state[0];
    p1.y = best_path_[i]->state[1];
    p1.z = best_path_[i]->state[2];

    p2.x = best_path_[it]->state[0];
    p2.y = best_path_[it]->state[1];
    p2.z = best_path_[it]->state[2];

    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
    vertex_marker.points.push_back(p1);
    vertex_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);
  marker_array.markers.push_back(vertex_marker);

  reposition_Path_Vis_Pub_.publish(marker_array);
}

void GraphManager::visualizeSparseGraph()
{
  std::shared_ptr<Graph> g = sparse_graph_->graph_;
  std::unordered_map<int, Vertex *> &v_map = sparse_graph_->vertices_map_;

  if (sparse_graph_->getNumVertices() == 0)
    return;

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
  edge_marker.color.r = 93 / 255.0;
  edge_marker.color.g = 142.0 / 255.0;
  edge_marker.color.b = 193.0 / 255.0;
  edge_marker.color.a = 1.0;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
  ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
  {
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
  vertex_marker.color.r = 93.0 / 255.0;
  vertex_marker.color.g = 142.0 / 255.0;
  vertex_marker.color.b = 193.0 / 255.0;
  vertex_marker.color.a = 1.0;
  // vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  visualization_msgs::Marker frontier_vertex_marker;
  frontier_vertex_marker.header.stamp = ros::Time::now();
  frontier_vertex_marker.header.seq = 0;
  frontier_vertex_marker.header.frame_id = "world";
  frontier_vertex_marker.id = 0;
  frontier_vertex_marker.ns = "frontier";
  frontier_vertex_marker.action = visualization_msgs::Marker::ADD;
  frontier_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  frontier_vertex_marker.scale.x = 0.4;
  frontier_vertex_marker.scale.y = 0.4;
  frontier_vertex_marker.scale.z = 0.4;
  frontier_vertex_marker.color.r = 255.0 / 255.0;
  frontier_vertex_marker.color.g = 0.0 / 255.0;
  frontier_vertex_marker.color.b = 0.0 / 255.0;
  frontier_vertex_marker.color.a = 0.7;
  frontier_vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    int id = g->getVertexProperty(it);
    if (v_map[id]->is_frontier)
    {

      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];

      frontier_vertex_marker.points.push_back(p1);
    }

    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];

    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);
  marker_array.markers.push_back(frontier_vertex_marker);

  sparse_Graph_Vis_Pub_.publish(marker_array);
}

void GraphManager::visualizeGlobalGraph()
{
  std::shared_ptr<Graph> g = global_graph_->graph_;
  std::unordered_map<int, Vertex *> &v_map = global_graph_->vertices_map_;

  if (global_graph_->getNumVertices() == 0)
    return;

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
  edge_marker.color.r = 0 / 255.0;
  edge_marker.color.g = 200.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
  ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
  {
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
  vertex_marker.color.r = 255.0 / 255.0;
  vertex_marker.color.g = 255.0 / 255.0;
  vertex_marker.color.b = 255.0 / 255.0;
  vertex_marker.color.a = 1.0;
  // vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  visualization_msgs::Marker frontier_vertex_marker;
  frontier_vertex_marker.header.stamp = ros::Time::now();
  frontier_vertex_marker.header.seq = 0;
  frontier_vertex_marker.header.frame_id = "world";
  frontier_vertex_marker.id = 0;
  frontier_vertex_marker.ns = "frontier";
  frontier_vertex_marker.action = visualization_msgs::Marker::ADD;
  frontier_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  frontier_vertex_marker.scale.x = 0.4;
  frontier_vertex_marker.scale.y = 0.4;
  frontier_vertex_marker.scale.z = 0.4;
  frontier_vertex_marker.color.r = 255.0 / 255.0;
  frontier_vertex_marker.color.g = 0.0 / 255.0;
  frontier_vertex_marker.color.b = 0.0 / 255.0;
  frontier_vertex_marker.color.a = 1.0;
  frontier_vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    int id = g->getVertexProperty(it);
    if (v_map[id]->is_frontier)
    {

      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];

      frontier_vertex_marker.points.push_back(p1);
    }

    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];

    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);
  marker_array.markers.push_back(frontier_vertex_marker);

  global_Graph_Vis_Pub_.publish(marker_array);
}

void GraphManager::visualizePoseGraph()
{
  std::shared_ptr<Graph> g = pose_graph_->graph_;
  std::unordered_map<int, Vertex *> &v_map = pose_graph_->vertices_map_;

  if (pose_graph_->getNumVertices() == 0)
    return;

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
  edge_marker.color.r = 240.0 / 255.0;
  edge_marker.color.g = 230.0 / 255.0;
  edge_marker.color.b = 130.0 / 255.0;
  edge_marker.color.a = 1.0;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
  ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
  {
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
  vertex_marker.color.r = 240.0 / 255.0;
  vertex_marker.color.g = 230.0 / 255.0;
  vertex_marker.color.b = 130.0 / 255.0;
  vertex_marker.color.a = 1.0;
  // vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];

    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  pose_Graph_Vis_Pub_.publish(marker_array);
}

void GraphManager::segmentPath(std::vector<Vertex *> path,
                               nav_msgs::Path &path_seg)
{
  float dt_ = 0.5;
  path_seg.poses.clear();
  double v_max_ = 0.5;
  double yaw_rate_max_ = 0.5;
  if (path.size() == 0)
    return;

  for (int i = 0; i < (path.size() - 1); ++i)
  {
    Eigen::Vector3d start(path[i]->state[0], path[i]->state[1],
                          path[i]->state[2]);
    Eigen::Vector3d end(path[i + 1]->state[0], path[i + 1]->state[1],
                        path[i + 1]->state[2]);
    Eigen::Vector3d distance = end - start;
    double yaw_start = 0;
    double yaw_end = 0;
    double yaw_direction = 0;
    if (yaw_direction > M_PI)
    {
      yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI)
    {
      yaw_direction += 2.0 * M_PI;
    }

    double dist_norm = distance.norm();
    double disc = std::min(dt_ * v_max_ / dist_norm,
                           dt_ * yaw_rate_max_ / abs(yaw_direction));

    bool int_flag = true;

    if (int_flag)
    {
      for (double it = 0.0; it <= 1.0; it += disc)
      {
        tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                           (1.0 - it) * start[1] + it * end[1],
                           (1.0 - it) * start[2] + it * end[2]);
        double yaw = yaw_start + yaw_direction * it;
        if (yaw > M_PI)
          yaw -= 2.0 * M_PI;
        if (yaw < -M_PI)
          yaw += 2.0 * M_PI;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position = pose.position;
        pose_stamped.pose.orientation = pose.orientation;
        path_seg.poses.push_back(pose_stamped);
      }
    }
  }
} // segmentPath

void GraphManager::visualize()
{

  // GraphManager::visualizeLocalGraph();
  // GraphManager::visualizeGlobalGraph();
}

std_msgs::ColorRGBA GraphManager::heightMapColor(double h)
{

  std_msgs::ColorRGBA color;
  color.a = 0.8;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
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
