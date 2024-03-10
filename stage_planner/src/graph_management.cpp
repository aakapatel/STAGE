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
  nh_private_.param("safety_margin", safety_margin, safety_margin);
  nh_private_.param("robot_size", robot_box_size, robot_box_size);
  nh_private_.param("robot_type", robot_type_, robot_type_);
  nh_private_.param("truncate_sample_Z", truncate_sample_Z_, truncate_sample_Z_);

  // Subscribers

  m_poseSubscriber = nh_.subscribe(m_odometry, 1, &GraphManager::odomCallback, this);

  // publishers

  m_frontierPub = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_nodes_vis", 10);
  m_modified_Path_Pub = nh_.advertise<nav_msgs::Path>("/modified_path", 10);

  local_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_planning_graph", 10);

  global_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/global_planning_graph", 10);

  path_Ways_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/path_ways_graph", 10);

  pose_Graph_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/pose_graph", 10);

  local_Paths_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/local_dijkstra_paths", 10);

  global_Paths_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/global_dijkstra_paths", 10);

  improved_global_Paths_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/improved_global_dijkstra_paths", 10);

  reposition_Path_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/global_repos_path", 10);

  tree_Vis_Pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/rrt_tree", 10);

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

  std::string ns = ros::this_node::getName();
  sensor_params_.loadParams(ns + "/SensorParams");
  sensor_name = sensor_params_.sensor_list[0];
  robot_params_.loadParams(ns + "/RobotParams");
  global_space_params_.loadParams(ns + "/BoundedSpaceParams/Global");
  local_space_params_.loadParams(ns + "/BoundedSpaceParams/Local");
  planning_params_.loadParams(ns + "/PlanningParams");
  
  geofence_manager_.reset(new GeofenceManager());
  geofence_manager_->loadParams(ns + "/GeofenceParams");
  // ROS_INFO_STREAM ("planning param traverse length max = " << planning_params_.traverse_length_max << "\n");

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
      GraphManager::resetSparseGraph ();
      GraphManager::resetGlobalGraph();
      GraphManager::resetPathWaysGraph();
      
      m_prevPose = *pose;
      firstTime = false;
      StateVec home_state;
      home_state[0] = m_prevPose.pose.pose.position.x;
      home_state[1] = m_prevPose.pose.pose.position.y;
      home_state[2] = m_prevPose.pose.pose.position.z;
      home_state[3] = tf::getYaw(m_prevPose.pose.pose.orientation);  

      Vertex *home_vertex = new Vertex(pose_graph_->generateVertexID(), home_state);
      pose_graph_->addVertex(home_vertex);
    }
  else
  {
    m_currentPose = *pose;

    StateVec newPoint(pose->pose.pose.position.x,
                      pose->pose.pose.position.y,
                      pose->pose.pose.position.z, 
                      tf::getYaw(pose->pose.pose.orientation));
    StateVec oldPoint(m_prevPose.pose.pose.position.x,
                      m_prevPose.pose.pose.position.y,
                      m_prevPose.pose.pose.position.z,
                      tf::getYaw(pose->pose.pose.orientation));

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
      global_vertices_ids_.push_back(pose_vertex->id);
      global_graph_->addVertex(pose_vertex);
        
      Vertex *path_vertex_root = new Vertex(++fake_id_count, robotState_);
      path_vertex_root->is_root = true;
      path_way_vertices.push_back(path_vertex_root);
      global_graph_->state_history_.push_back(robotState_);
      
      // GraphManager::updateSparseGraph();
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
  yaw_ = yaw * (180 / 3.14159);
  
  robotState_[0] = pose->pose.pose.position.x;
  robotState_[1] = pose->pose.pose.position.y;
  robotState_[2] = pose->pose.pose.position.z;
  robotState_[3] = tf::getYaw(pose->pose.pose.orientation);
  
  robotStateWithYaw_[0] = pose->pose.pose.position.x;
  robotStateWithYaw_[1] = pose->pose.pose.position.y;
  robotStateWithYaw_[2] = pose->pose.pose.position.z;
  robotStateWithYaw_[3] = tf::getYaw(pose->pose.pose.orientation);

  GraphManager::visualizePoseGraph();
}

double GraphManager::calculateDistance(StateVec s1,
                                       StateVec s2)
{

  double d =
    sqrt(pow(s1.x() - s2.x(), 2) + pow(s1.y() - s2.y(), 2) +
         pow(s1.z() - s2.z(), 2));
  return d;
}

void GraphManager::computeVolumetricGainRayCasting(StateVec& state, VolumetricGain& vgain) {

  vgain.reset();

  std::vector<std::tuple<int, int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>> voxel_log;
  int raw_unk_voxels_count = 0;

  Eigen::Vector3d origin(state[0], state[1], state[2]);
  std::tuple<int, int, int> gain_log_tmp;
  std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>>
  voxel_log_tmp;
  std::vector<Eigen::Vector3d> multiray_endpoints;
  sensor_params_.sensor[sensor_name].getFrustumEndpoints(state,
                                                         multiray_endpoints);

  map_manager_->getScanStatus(origin, multiray_endpoints, gain_log_tmp,
                              voxel_log_tmp,
                              sensor_params_.sensor[sensor_name]);
  int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0,
  num_unknown_surf_voxels = 0;

  for (auto& vl : voxel_log_tmp) {
    Eigen::Vector3d voxel = vl.first;
    MapManager::VoxelStatus vs = vl.second;
    if (vs == MapManager::VoxelStatus::kUnknown) ++raw_unk_voxels_count;
    // valid voxel.
    if (vs == MapManager::VoxelStatus::kUnknown) {
      ++num_unknown_voxels;
    } else if (vs == MapManager::VoxelStatus::kFree) {
      ++num_free_voxels;
    } else if (vs == MapManager::VoxelStatus::kOccupied) {
      ++num_occupied_voxels;
    } else {
      ROS_ERROR("Unsupported voxel type.");
    }
  }
  gain_log.push_back(std::make_tuple(num_unknown_voxels, num_free_voxels,
                                     num_occupied_voxels,
                                     num_unknown_surf_voxels));
  
  // Check if it is a potential frontier.
  if (sensor_params_.sensor[sensor_name].isFrontier(
    num_unknown_voxels * map_manager_->getResolution(), vgain.unknown_percentage)) {
    vgain.is_frontier = true;  // Event E2
    // vgain.is_evaluated = false;
  }

  // Return gain values.
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    vgain.gain += (num_unknown_voxels * 60);
  }

}

double GraphManager::computeExplorationGain(StateVec state)
{

  std::vector<Eigen::Vector3d> free, occupied, unknown;
  GraphManager::extractVoxelsInsideFrustum(state, free, occupied, unknown);
  // double total_voxels = (free.size() + occupied.size() + unknown.size());
  double gain = 0;
  
  for (int i = 0; i != unknown.size(); i++) { 

    // Eigen::Vector3d directionaToVertex = (unknown[i] - robotState_).normalized();
    // Eigen::Vector3d directionalVector = (state - robotState_).normalized();
    // double angle = acos(directionalVector.dot(directionaToVertex) / (directionalVector.norm() * directionaToVertex.norm() )) * 180/M_PI;
    Eigen::Vector3d state_vec (state[0], state[1], state[2]); 
    if (MapManager::VoxelStatus::kOccupied !=
      map_manager_->getPathStatus(unknown[i], state_vec, robot_box_size_, false)) {
      gain = gain + 1;
    }
  }
  
  gain = 100 * gain * std::pow(map_manager_->getResolution(), 3);
  
  return gain;
}



void GraphManager::extractVoxelsInsideFrustum (StateVec state, 
                                               std::vector<Eigen::Vector3d> &free,
                                               std::vector<Eigen::Vector3d> &occupied,
                                               std::vector<Eigen::Vector3d> &unknown) {
  
  free.clear();
  occupied.clear();
  unknown.clear();
  Eigen::Vector3d box_size (1, 1, 1);
  std::vector<Eigen::Vector3d> occupied_voxels;
  std::vector<Eigen::Vector3d> free_voxels;
  std::vector<Eigen::Vector3d> unknown_voxels;
  
  free_voxels.clear();
  occupied_voxels.clear();

  Eigen::Vector3d state_vec (state[0], state[1], state[2]); 
  map_manager_->extractLocalMap(state_vec, box_size, occupied_voxels, free_voxels, unknown_voxels);
  
  double hFOV = 2*M_PI;
  double vFOV = M_PI/4;
  double range = 1;
  
  // TODO @ can use OMP to parallalize the following 3 loops 

  for (Eigen::Vector3d voxel : unknown_voxels) {
    
    Eigen::Vector3d state_vec (state[0], state[1], state[2]);
    Eigen::Vector3d toPoint = voxel - state_vec;
    double h_angle = std::atan2(toPoint.y(), toPoint.x());
    double v_angle = std::atan2(toPoint.z(), toPoint.norm());

    // if (std::abs(h_angle) <= hFOV / 2 and 
    if (std::abs(v_angle) <= vFOV / 2 and 
        toPoint.norm() <= range) {
      
        unknown.push_back(voxel);
    }
  }

  // for (Eigen::Vector3d voxel : free_voxels) {
  //
  //   Eigen::Vector3d toPoint = voxel - state;
  //   double h_angle = std::atan2(toPoint.y(), toPoint.x());
  //   double v_angle = std::atan2(toPoint.z(), toPoint.norm());
  //
  //   if (std::abs(h_angle) <= hFOV / 2 and 
  //       std::abs(v_angle) <= vFOV / 2 and 
  //       toPoint.norm() <= range) {
  //     
  //       free.push_back(voxel);
  //   }
  // }
  //
  // for (Eigen::Vector3d voxel : occupied_voxels) {
  //
  //   Eigen::Vector3d toPoint = voxel - state;
  //   double h_angle = std::atan2(toPoint.y(), toPoint.x());
  //   double v_angle = std::atan2(toPoint.z(), toPoint.norm());
  //
  //   if (std::abs(h_angle) <= hFOV / 2 and 
  //       std::abs(v_angle) <= vFOV / 2 and 
  //       toPoint.norm() <= range) {
  //     
  //       occupied.push_back(voxel);
  //   }
  // }

}

bool GraphManager::pointInsideFrustumVolume (StateVec state) {
  
  double hFOV = 2*M_PI;
  double vFOV = M_PI/4;
  double range = bounding_box_size_;
  
  Eigen::Vector3d robot_state (robotState_[0], robotState_[1], robotState_[2]);

  Eigen::Vector3d state_vec (state[0], state[1], state[2]);
  Eigen::Vector3d toPoint = state_vec - robot_state;
  double h_angle = std::atan2(toPoint.y(), toPoint.x());
  double v_angle = std::atan2(toPoint.z(), toPoint.norm());

  // if (std::abs(h_angle) <= hFOV / 2 and 
  if (std::abs(h_angle) <= hFOV / 2 and 
    std::abs(v_angle) <= vFOV / 2 and 
    toPoint.norm() <= range) {

      return true;
    } else {
    return false;
  }

}

void GraphManager::sampleNodes(std::vector<node *> &nodes, StateVec state, int iterations_max, double bounding_box_size)
{

  nodes.clear();

  node *current_pos = new node(state.x(), state.y(), state.z());
  current_pos->addParent(nullptr);
  nodes.push_back(current_pos);

  Eigen::Vector3d center (robotState_[0], robotState_[1], robotState_[2]);
  Eigen::Vector3d bb_size (bounding_box_size_, bounding_box_size_, bounding_box_size_);
  std::vector<Eigen::Vector3d> occupied_voxels;
  std::vector<Eigen::Vector3d> free_voxels;
  std::vector<Eigen::Vector3d> unknown_voxels;
   
  map_manager_->extractLocalMap(center, bb_size, occupied_voxels, free_voxels, unknown_voxels);
  
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
        // Eigen::Vector3d r_node = *((*it_node)->point); 
        // double angle = atan2(r_node[1] - (random_state.y()), (r_node[0] - random_state.x())) * (180 / M_PI); 
        // if (angle < m_local_fov or angle > - m_local_fov) 
        // {
          Eigen::Vector3d direction = random_state - *((*it_node)->point);
          double new_distance = abs(direction.norm());
          if (new_distance < distance)
          {
            distance = new_distance;
            parent = *it_node;
          }
        // }
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
  
  // GraphManager::visualizeTree(nodes);
  // ROS_INFO_STREAM ("Sampled : " << nodes.size() << " nodes" << "\n ");

} // sampleNodes


StateVec GraphManager::eigenToState(Eigen::Vector3d v)
{
  StateVec s (v[0], v[1], v[2], 0);
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
    StateVec home_state;
    home_state[0] = robotState_[0];
    home_state[1] = robotState_[1];
    home_state[2] = robotState_[2] + 1;
    home_state[2] = robotState_[3];
    home = new Vertex(assignVertexID(), home_state);
    GraphManager::insertVertexKdtree(home);
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

void GraphManager::resetPathWaysGraph()
{

  if (path_ways_graph_ != NULL)
  {
    path_ways_graph_->reset();
  }
  path_ways_graph_.reset(new GraphUtils());
  path_ways_graph_rep_.reset();
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

void GraphManager::orientVertices (std::shared_ptr<GraphUtils> graph, const ShortestPathsReport& rep) 
{

  // Assign the heading angle to the vertices based on shortest path report.
  int num_vertices = graph->getNumVertices();
  for (int id = 1; id < num_vertices; ++id) {
    int pid = graph->getParentIDFromShortestPath(id, rep);
    Vertex* v = graph->getVertex(id);
    Vertex* vp = graph->getVertex(pid);
    Eigen::Vector3d vec(v->state[0] - vp->state[0],
                        v->state[1] - vp->state[1],
                        v->state[2] - vp->state[2]);
    v->state[3] = std::atan2(vec[1], vec[0]);
  }

}

bool GraphManager::getNeighbours(const StateVec *state, double range, std::vector<Vertex *> *v_res)
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

void GraphManager::updateKdtree(std::shared_ptr<GraphUtils> graph)
{
  
  if (graph->getNumVertices() == 0) {
    return;
  }

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;

  std::pair<Graph::GraphType::edge_iterator,
  Graph::GraphType::edge_iterator>
  ei;
  graph->graph_->getVertexIterator(vi);
  graph->graph_->getEdgeIterator(ei);

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = graph->graph_->getVertexProperty(it);
    if (graph->vertices_map_[id]->is_frontier)
    {
      graph->vertices_map_[id]->is_frontier = false;

      std::vector<Vertex *> knn;
      graph->getNearestVertices(&graph->vertices_map_[id]->state, 2, &knn);
      
      // can assign some checks here as this function is currently not being used elsewhere. 

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

void GraphManager::clusterNodes (std::shared_ptr<GraphUtils> g) 
{
  if (g->getNumVertices() == 0) {
    return;
  }
  // cluster frontier nodes based on relative volumetric gain for efficient insertion into sparse graph  
  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  g->graph_->getVertexIterator(vi);

  int cluster_count = 0;

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = g->graph_->getVertexProperty(it);
    StateVec v_s = g->vertices_map_[id]->state;
    Vertex * v = g->vertices_map_[id];
    std::vector<Vertex *> nearby_vertices;
    double my_gain = -std::numeric_limits<double>::max();
    g->getNearestVertices(&g->vertices_map_[id]->state, knn_range_, &nearby_vertices);
    
    Vertex * centroid_vertex;
    for (int i = 0; i < nearby_vertices.size() and nearby_vertices[i]->is_leaf_vertex; i++)
    {
      
      Vertex * nearby_vertex = nearby_vertices[i];
      if (nearby_vertex->cluster_id == -1 and nearby_vertex->vol_gain.gain > my_gain) {
        
        my_gain = nearby_vertex->vol_gain.gain;
        centroid_vertex->cluster_id = cluster_count;
        centroid_vertex = nearby_vertex;

      } else {
        nearby_vertex->cluster_id = cluster_count;
      }

    }
    centroid_vertex->is_cluster_centroid = true;
    cluster_count = cluster_count + 1;
  }

}


bool GraphManager::buildLocalGraph()
{
  
  bool graph_status = false;
  int frontier_vertices_count = 0;
  // high_resolution_clock::time_point start_total = high_resolution_clock::now();

  GraphManager::resetLocalGraph();

  local_root_vertex = new Vertex(sub_graph_->generateVertexID(), robotState_);
  local_root_vertex->is_root = true;
  sub_graph_->addVertex(local_root_vertex);
  
  // Insert the sub graph vertedx to global graph to re-use the SG
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

  for (int i = 0; i < nodes.size(); ++i)
  {
    Vertex *vertex = new Vertex(sub_graph_->generateVertexID(), eigenToState(*(nodes[i]->point)));
    sub_graph_->addVertex(vertex);
  }

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  sub_graph_->graph_->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {

    int id = sub_graph_->graph_->getVertexProperty(it);
    Vertex* vertex = sub_graph_->vertices_map_[id];
    
    // Get nearby vertices from the current vertex and connect the them to form local edges
    std::vector<Vertex *> nearby_vertices;
    sub_graph_->getNearestVertices(&sub_graph_->vertices_map_[id]->state, knn_range_, &nearby_vertices);

    for (int i = 0; i < nearby_vertices.size(); i++)
    {
      
      Vertex* neighbour = nearby_vertices[i]; 

      int num_edges = vertex->edges;
      double weight = (vertex->state - neighbour->state).norm();

      if (num_edges < num_edges_max_ and weight < max_edge_length_)
      {

        // CHECK FOR COLLISION FREE EDGE

        MapManager::VoxelStatus vn = map_manager_->getPathStatus(vertex->state_3d, neighbour->state_3d, robot_box_size_, true);

        if (MapManager::VoxelStatus::kFree == vn)
        {

          sub_graph_->addEdge(vertex, neighbour, weight);
          vertex->edges++;
        }
      }
    }
  }

  // high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  //
  // auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  //
  // ROS_INFO_STREAM("Local Graph build time " << duration_total.count() << " ms \n");
  
  ROS_DEBUG_STREAM("Sub graph has " << sub_graph_->getNumVertices() << " nodes & " << sub_graph_->getNumEdges() << " edges\n");
  
  if (sub_graph_->getNumVertices() != 0 && sub_graph_->getNumEdges() != 0) {
    
    // GraphManager::visualizeLocalGraph();
    graph_status = true;
  }


  return graph_status;

} // buildLocalGraph

bool GraphManager::extractShortestPaths (std::shared_ptr<GraphUtils> graph, ShortestPathsReport rep, Vertex* root, 
                                         std::vector<std::vector<Vertex *>> &dijkstra_paths) 
{
  
  bool path_status = false;
  // Fetch shortest path report from root vertex to all vertices in the graph
  if (!graph->findShortestPaths(root->id, rep))
  {
    ROS_WARN("Could not find dijkstra shortest paths on the graph \n");
    return path_status;
  } 
  // else {
    // GraphManager::orientVertices(graph, rep);
  // }
  
  dijkstra_paths.clear();
  graph->findLeafVertices(rep);
 
  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator> vi;
  graph->graph_->getVertexIterator(vi);

  if (graph->leaf_vertices_ids_.empty()) {
    return false;
  }
  for (int id : graph->leaf_vertices_ids_)
  {
    Vertex * vertex = graph->vertices_map_[id];
    
    GraphManager::computeVolumetricGainRayCasting(vertex->state, vertex->vol_gain);
    
    /*

    if (!vertex->is_evaluated) {
      
      // compute and assign volumetric gain to the vertex
      // Assign same gain and frontier status to the neighbours for efficient expansion
      GraphManager::computeVolumetricGainRayCasting(vertex->state, vertex->vol_gain);
      std::vector<Vertex *> nearby_vertices;
      graph->getNearestVertices(&graph->vertices_map_[id]->state, knn_range_, &nearby_vertices);

      for (int i = 0; i < nearby_vertices.size(); i++)
      {

        Vertex* neighbour = nearby_vertices[i]; 
        neighbour->vol_gain.gain = vertex->vol_gain.gain;
        neighbour->vol_gain.is_frontier = vertex->vol_gain.is_frontier;
        if (vertex->vol_gain.is_frontier) {

          vertex->is_evaluated = false;
          neighbour->is_evaluated = false;
        } else {

          vertex->is_evaluated = true;
          neighbour->is_evaluated = true;
        }
      }
    }
  
   */
  
  }

  // cluster the nodes based on their volumetric gain 
    
  GraphManager::clusterNodes(graph); 
  
  for (int id : graph->leaf_vertices_ids_)
  {
    Vertex * vertex = graph->vertices_map_[id];
    
    if (vertex->vol_gain.is_frontier and vertex->is_cluster_centroid)
    // if (vertex->vol_gain.is_frontier)
    {
      std::vector<Vertex *> dijkstra_path;
      dijkstra_path.clear();

      graph->getShortestPath(vertex->id, rep, true, dijkstra_path);

      if (!dijkstra_path.empty() and getPathLength(dijkstra_path) > 4)
      {
        dijkstra_paths.push_back(dijkstra_path);
      }
    }

  }
  

  if (!dijkstra_paths.empty()) {
    path_status = true;
  } else {
    return false;
  }
  
  return path_status;

}

bool GraphManager::planLocalPath(std::vector<std::vector<Vertex *>> dijkstra_paths)
{
  
  bool path_status = true;
  local_paths_.clear();

  if (!dijkstra_paths.empty()) {

    for (std::vector<Vertex *> dijkstra_path : dijkstra_paths) {
      
      if (!dijkstra_path.empty())
      {
        Eigen::Vector3d path_goal (dijkstra_path.back()->state[0], dijkstra_path.back()->state[1], dijkstra_path.back()->state[2]);

        if (isLocal(path_goal)) {

          std::pair<std::vector<Vertex *>, double> p = std::make_pair(dijkstra_path, getPathLength(dijkstra_path));

          local_paths_.push_back(p);
        }
      }
    }

  } else {
    ROS_WARN_STREAM ("Sub graph -> No dijkstra paths :( \n");
    return false;
  }

  if (!local_paths_.empty())
  {
    GraphManager::visualizeLocalPaths(local_paths_);

    std::vector<Vertex *> path;
    double gain, path_length, reward;
    double my_reward = -std::numeric_limits<double>::max();

    for (auto i = 0; i < local_paths_.size(); i++)
    {

      gain = 100 * local_paths_[i].first.back()->vol_gain.unknown_percentage;
      
      path_length = local_paths_[i].second;
      
      // ROS_INFO_STREAM ("Vertex Gain --> " << gain << "\t Path length --> " << path_length << "\n");

      reward = gain/(path_length + 2);
      if (reward > my_reward)
      {
        my_reward = reward;
        path = local_paths_[i].first;
      }
    }

    best_path_.clear();
    GraphManager::improvePath(path, best_path_);
    std::vector<geometry_msgs::Pose> original_path;
    std::vector<geometry_msgs::Pose> modified_path;
    original_path.clear();
    modified_path.clear();
    geometry_msgs::Pose path_pose;
    for (int i = 0; i != best_path_.size(); ++i) {
      path_pose.position.x = best_path_[i]->state[0];
      path_pose.position.y = best_path_[i]->state[1];
      path_pose.position.z = best_path_[i]->state[2];
      path_pose.orientation.x = 0;
      path_pose.orientation.y = 0;
      path_pose.orientation.z = 0.2;
      path_pose.orientation.w = 1;
      original_path.push_back(path_pose);
    }

    bool improvement_success = GraphManager::improveFreePath (original_path, modified_path, true);
    
    if (improvement_success) {
      geometry_msgs::PoseStamped pose_mod;
      pose_mod.header.frame_id = m_worldFrameId;
      pose_mod.header.stamp = ros::Time::now();
      mod_path_.header.frame_id = m_worldFrameId;
      mod_path_.header.stamp = ros::Time::now();

      for (int i = 0; i != modified_path.size(); ++i){
        pose_mod.pose.position.x = modified_path[i].position.x;
        pose_mod.pose.position.y = modified_path[i].position.y;
        pose_mod.pose.position.z = modified_path[i].position.z;
        mod_path_.poses.push_back(pose_mod);
      }
    
      m_modified_Path_Pub.publish(mod_path_);
    } 
    if (mod_path_.poses.empty()) 
    {
      ROS_WARN("Sub graph -> Improved local path is empty \n");
      path_status = false;
      return path_status;
    }
  } 
  else 
  {
    ROS_WARN("Sub graph -> No shortest paths to local frontier vertices \n");
    path_status = false;
    return path_status;
  }

  return path_status;

} // planLocalPath

void GraphManager::updateSparseGraph() 
{
  
  if(sparse_graph_->getNumVertices() == 0) {
    return;
  }

  Vertex * new_vertex = new Vertex (sparse_graph_->generateVertexID(), robotState_);
  new_vertex->is_root = true;
  sparse_graph_->addVertex(new_vertex);
 
  std::vector<Vertex *> nearby_vertices;
  sparse_graph_->getNearestVertices(&new_vertex->state, knn_range_, &nearby_vertices);

  for (int i = 0; i < nearby_vertices.size(); ++i)
  {
    Vertex* neighbour = nearby_vertices[i];
    int num_edges = new_vertex->edges;
    double weight = (new_vertex->state - neighbour->state).norm();

    // if (num_edges < num_edges_max_ and weight < max_edge_length_)
    if (weight < max_edge_length_ and weight > 0.2)
    {

      if (!sparse_graph_->edgeExist(new_vertex->id, neighbour->id) and 
        MapManager::VoxelStatus::kFree == map_manager_->getPathStatus(new_vertex->state_3d, neighbour->state_3d, robot_box_size_, true))
      {

        sparse_graph_->addEdge(new_vertex, neighbour, weight);
        new_vertex->edges++;
      }
    }
  }

  GraphManager::visualizeSparseGraph();

}

void GraphManager::insertBestPathsToSparseGraph (std::vector<std::vector<Vertex *>> dijkstra_paths) 
{

  bool graph_status = true;
  if (dijkstra_paths.empty()) {
    ROS_WARN("Dijkstra paths are empty. Can not insert paths into sparse graph \n");
    return;
  }

  std::vector<Vertex *> newly_added_vertices;
  newly_added_vertices.clear();
  
  Vertex * vertex = new Vertex (sparse_graph_->generateVertexID(), robotState_);
  vertex->is_root = true;
  sparse_graph_->addVertex(vertex);
  newly_added_vertices.push_back(vertex);

  for (std::vector<Vertex *> dijkstra_path : dijkstra_paths) {

    for (int i = 0; i < dijkstra_path.size(); ++i) {

      Vertex* v = dijkstra_path[i];

      if (!sparse_graph_->vertexExist(v->state)) {

        std::vector<std::pair<int, double>> edge_pairs = sub_graph_->edge_map_[v->id];
        Vertex* new_vertex = new Vertex(sparse_graph_->generateVertexID(), v->state);
        // new_vertex->vol_gain = v->vol_gain;
        sparse_graph_->addVertex(new_vertex);
        newly_added_vertices.push_back(new_vertex);

        for (std::pair<int, double> edge_pair : edge_pairs) {

          int id = edge_pair.first;
          Vertex* neighbour_vertex = sub_graph_->vertices_map_[id];

          if (std::find(dijkstra_path.begin(), dijkstra_path.end(), neighbour_vertex) != dijkstra_path.end()) {

            if (!sparse_graph_->vertexExist(neighbour_vertex->state)) {

              Vertex* new_neighbour_vertex = new Vertex (sparse_graph_->generateVertexID(), neighbour_vertex->state);
              new_neighbour_vertex->vol_gain = neighbour_vertex->vol_gain;
              sparse_graph_->addVertex(new_neighbour_vertex);
              newly_added_vertices.push_back(new_vertex);
              
              double weight = (new_vertex->state - new_neighbour_vertex->state).norm();
              sparse_graph_->addEdge(new_vertex, new_neighbour_vertex, weight);

            }
          }
        }
      }
    }
  }
  
  high_resolution_clock::time_point start_total = high_resolution_clock::now();

  for (Vertex* new_vertex : newly_added_vertices)
  {

    // Get nearby vertices from the current vertex and connect the them to form local edges

    // GraphManager::computeVolumetricGainRayCasting(new_vertex->state, new_vertex->vol_gain);
    
    std::vector<Vertex *> nearby_vertices;
    sparse_graph_->getNearestVertices(&new_vertex->state, knn_range_, &nearby_vertices);

    for (int i = 0; i < nearby_vertices.size(); ++i)
    {
      Vertex* neighbour = nearby_vertices[i];
      int num_edges = new_vertex->edges;
      double weight = (new_vertex->state - neighbour->state).norm();

      // if (num_edges < num_edges_max_ and weight < max_edge_length_)
      if (weight < max_edge_length_ and weight > 0.2)
      {

        // if (!sparse_graph_->edgeExist(new_vertex->id, neighbour->id) and 
        //   MapManager::VoxelStatus::kFree == map_manager_->getPathStatus(new_vertex->state_3d, neighbour->state_3d, robot_box_size_, true))
        if ( 
          MapManager::VoxelStatus::kFree == map_manager_->getPathStatus(new_vertex->state_3d, neighbour->state_3d, robot_box_size_, true))
        {

          sparse_graph_->addEdge(new_vertex, neighbour, weight);
          new_vertex->edges++;
        }
      }
    }
  } 
  
  // GraphManager::clusterNodes(sparse_graph_); 
  global_root_vertex = vertex;
  
  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  ROS_INFO_STREAM("Principle paths insertion time \t " << duration_total.count() << " ms \n");
  GraphManager::visualizeSparseGraph();
  
}

bool GraphManager::expandSparseGraph () {

  bool graph_status = true;
  if (sparse_graph_->getNumVertices() == 0) {
    ROS_WARN("Sparse graph empty \n");
    return false;
  } 
  
  std::vector<Vertex *> newly_added_vertices;
  newly_added_vertices.clear();
  
  Vertex * r_vertex = new Vertex (sparse_graph_->generateVertexID(), robotState_);
  r_vertex->is_root = true;
  // r_vertex->is_leaf_vertex = true;
  sparse_graph_->addVertex(r_vertex);
  newly_added_vertices.push_back(r_vertex);
  
  // Expand the sparse graph by adding sampled nodes around robot pose 
  std::vector<node *> nodes;
  nodes.clear();

  GraphManager::sampleNodes(nodes, robotState_, iterations_local_/10, bounding_box_size_);
  if (nodes.empty())
  {
    graph_status = false;
    ROS_WARN("Could not sample nodes in traversable space \n");
    return graph_status;
  }
  
  // Add randomly sampled nodes to sparse graph 
 
  for (int i = 0; i < nodes.size(); ++i)
  {
    Vertex *vertex = new Vertex(sparse_graph_->generateVertexID(), eigenToState(*(nodes[i]->point)));
    sparse_graph_->addVertex(vertex);
    newly_added_vertices.push_back(vertex);
  }
  
  for (Vertex* new_vertex : newly_added_vertices)
  {

    // Get nearby vertices from the current vertex and connect the them to form local edges
    std::vector<Vertex *> nearby_vertices;
    sparse_graph_->getNearestVertices(&new_vertex->state, knn_range_, &nearby_vertices);

    for (int i = 0; i < nearby_vertices.size(); ++i)
    {
      Vertex* neighbour = nearby_vertices[i];
      int num_edges = new_vertex->edges;
      double weight = (new_vertex->state - neighbour->state).norm();

      // if (num_edges < num_edges_max_ and weight < max_edge_length_)
      if (weight < max_edge_length_)
      {

        if (!sparse_graph_->edgeExist(new_vertex->id, neighbour->id) and 
          MapManager::VoxelStatus::kFree == map_manager_->getPathStatus(new_vertex->state_3d, neighbour->state_3d, robot_box_size_, true))
        {

          sparse_graph_->addEdge(new_vertex, neighbour, weight);
          new_vertex->edges++;
        }
      }
    }
  }

  global_root_vertex = r_vertex;
  
  GraphManager::visualizeSparseGraph();

  return graph_status;
}



bool GraphManager::planGlobalPath(std::vector<std::vector<Vertex *>> dijkstra_paths)
{

  bool path_status = true;
  global_paths_.clear();

  if (!dijkstra_paths.empty()) {

    for (std::vector<Vertex *> dijkstra_path : dijkstra_paths) {
      
      if (!dijkstra_path.empty())
      {
        Eigen::Vector3d path_goal (dijkstra_path.back()->state[0], dijkstra_path.back()->state[1], dijkstra_path.back()->state[2]);

        std::pair<std::vector<Vertex *>, double> p = std::make_pair(dijkstra_path, getPathLength(dijkstra_path));

        global_paths_.push_back(p);
      }
    }

  } else {
    ROS_WARN_STREAM ("Global graph -> No dijkstra paths :( \n");
    return false;
  }

  if (!global_paths_.empty())
  {

    GraphManager::visualizeGlobalPaths(global_paths_);

    std::vector<Vertex *> path;

    double gain, path_length, reward;
    double my_reward = -std::numeric_limits<double>::max();

    for (auto i = 0; i < global_paths_.size(); i++)
    {

      gain = 100 * global_paths_[i].first.back()->vol_gain.unknown_percentage;
      path_length = global_paths_[i].second;

      reward = gain - (path_length);
      if (reward > my_reward)
      {
        my_reward = reward;
        path = global_paths_[i].first;
      }
    }

    best_path_.clear();
    GraphManager::improvePath(path, best_path_);
    

    std::vector<geometry_msgs::Pose> original_path;
    std::vector<geometry_msgs::Pose> modified_path;
    original_path.clear();
    modified_path.clear();
    geometry_msgs::Pose path_pose;
    for (int i = 0; i != best_path_.size(); ++i) {
      path_pose.position.x = best_path_[i]->state[0];
      path_pose.position.y = best_path_[i]->state[1];
      path_pose.position.z = best_path_[i]->state[2];
      path_pose.orientation.x = 0;
      path_pose.orientation.y = 0;
      path_pose.orientation.z = 0.2;
      path_pose.orientation.w = 1;
      original_path.push_back(path_pose);
    }

    bool improvement_success = GraphManager::improveFreePath (original_path, modified_path, true);
    
    if (improvement_success) {
      geometry_msgs::PoseStamped pose_mod;
      
      pose_mod.header.frame_id = m_worldFrameId;
      pose_mod.header.stamp = ros::Time::now();
      mod_path_.header.frame_id = m_worldFrameId;
      mod_path_.header.stamp = ros::Time::now();

      for (int i = 0; i != modified_path.size(); ++i){
        pose_mod.pose.position.x = modified_path[i].position.x;
        pose_mod.pose.position.y = modified_path[i].position.y;
        pose_mod.pose.position.z = modified_path[i].position.z;
        mod_path_.poses.push_back(pose_mod);
      }
    
      m_modified_Path_Pub.publish(mod_path_);
    } 


    if (mod_path_.poses.empty())
    {
      path_status = false;
      ROS_WARN("GLOBAL GRPH -> Improved path is empty \n");
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
  if (mod_path_.poses.empty()) {
    path_set = false;
    return path_set;
  } else {

    GraphManager::visualizeRepositionPath();
    nav_msgs::Path path_seg;
    path_seg.poses.clear();
    Eigen::Vector3d v_root;
    if (path_iterator_ < mod_path_.poses.size())
    {

      if (path_iterator_ == 0)
      {
        v_root = Eigen::Vector3d(robotState_[0], robotState_[1], robotState_[2]);
      }
      else
    {
        v_root = Eigen::Vector3d(mod_path_.poses[path_iterator_ - 1].pose.position.x, mod_path_.poses[path_iterator_ - 1].pose.position.y, mod_path_.poses[path_iterator_ - 1].pose.position.z);
      }

      Eigen::Vector3d v_vert(mod_path_.poses[path_iterator_].pose.position.x, mod_path_.poses[path_iterator_].pose.position.y, mod_path_.poses[path_iterator_].pose.position.z);
      MapManager::VoxelStatus vs = map_manager_->getPathStatus(v_root, v_vert, robot_box_size_, true);

      if (MapManager::VoxelStatus::kFree == vs or MapManager::VoxelStatus::kUnknown == vs)
      {

        geometry_msgs::PoseStamped pose_root;
        geometry_msgs::PoseStamped pose_next;
        pose_root.pose.position.x = v_root[0];
        pose_root.pose.position.y = v_root[1];
        pose_root.pose.position.z = v_root[2];
        pose_next.pose.position.x = v_vert[0];
        pose_next.pose.position.y = v_vert[1];
        pose_next.pose.position.z = v_vert[2];

        path_seg.poses.push_back(pose_root);
        path_seg.poses.push_back(pose_next);
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
        mod_path_.poses.clear();
        best_path_.clear();
        path_set =  false;
        return path_set;
      }
    }
    else
  {
      ROS_DEBUG_STREAM("Reached end of path \n");
      exploration_path_.poses.clear();
      reached_end_of_path_ = true;
      path_iterator_ = 0;
      mod_path_.poses.clear();
      best_path_.clear();
      path_set =  false;
      return path_set;
    }
  }

  return path_set;

}



bool GraphManager::modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
                     Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                     Eigen::Vector3d& p1_mod) {
  p1_mod = p1;

  Eigen::Vector3d p_center;
  p_center = (p0 + p1) / 2.0;
  Eigen::Vector3d p_dir;
  p_dir = (p1 - p0);
  double radius = p_dir.norm() / 2.0;
  Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  Eigen::Quaternion<double> quat_W2S;
  // Use the spherical with Cartesian (Forward, left, up) coordinate
  Eigen::Vector3d p_dir_norm = p_dir.normalized();
  double yaw_angle = std::atan2(p_dir_norm.y(), p_dir_norm.x());
  double pitch_angle =
      -std::atan2(p_dir_norm.z(), std::sqrt(p_dir_norm.x() * p_dir_norm.x() +
                                            p_dir_norm.y() * p_dir_norm.y()));
  quat_W2S = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY());

  pcl::PointCloud<pcl::PointXYZ>* pcl_tf(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Translation<double, 3> trans_W2S(p_center);
  Eigen::Transform<double, 3, Eigen::Affine> tf_W2S(trans_W2S * quat_W2S);
  pcl::transformPointCloud(*obstacle_pcl, *pcl_tf, tf_W2S.inverse());

  // add a local bounding box
  double kDx = robot_params_.safety_extension[0];
  double kDy = robot_params_.safety_extension[1];
  double kDz = robot_params_.safety_extension[2];

  // 6 rectanges in form:  ax+by+cz = 1
  std::vector<Eigen::Vector3d> u_l;
  std::vector<Eigen::Vector3d> p_l;
  u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  p_l.push_back(Eigen::Vector3d(-radius - kDx, 0.0, 0.0));
  p_l.push_back(Eigen::Vector3d(radius + kDx, 0.0, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, -kDy, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, kDy, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, 0.0, -kDz));
  p_l.push_back(Eigen::Vector3d(0.0, 0.0, kDz));
  std::vector<Eigen::Vector3d> hyperplane_list;
  std::vector<Eigen::Vector3d> tangent_point_list;
  for (int i = 0; i < 6; ++i) {
    Eigen::Vector3d a_l;
    a_l = u_l[i] / (u_l[i].dot(p_l[i]));
    tangent_point_list.push_back(p_l[i]);
    hyperplane_list.push_back(a_l);
  }

  // Keep points inside the local box only
  pcl::PointCloud<pcl::PointXYZ>* pcl_in_box(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
    // Check all 6 hyperplanes
    const double kDSign = 0.05;  // numeric issue
    double sign;
    int i = 0;
    for (i = 0; i < 6; ++i) {
      sign = p->x * hyperplane_list[i].x() + p->y * hyperplane_list[i].y() +
             p->z * hyperplane_list[i].z() - 1;
      if (sign > kDSign) break;
    }
    if (i == 6) {
      // inside the local box
      pcl_in_box->push_back(*p);
    }
  }
  if (pcl_in_box->size())
    pcl::copyPointCloud(*pcl_in_box, *pcl_tf);
  else {
    // full free space --> keep current vertex.
    return true;
  }

  // Find closest point
  double dist_min_sq = std::numeric_limits<double>::max();
  Eigen::Vector3d p_tangent;
  for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
    double dist_t = p->x * p->x + p->y * p->y + p->z * p->z;
    if (dist_t < dist_min_sq) {
      dist_min_sq = dist_t;
      p_tangent << p->x, p->y, p->z;
    }
  }

  const double kDDist = 0.01;  // deal with numeric error.
  if ((dist_min_sq == std::numeric_limits<double>::max()) ||
      (dist_min_sq < kDDist)) {
    // the path is too close to obstacle.
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[IMPRV] Path too close to obstacle");
    return false;
  }

  double a = radius, b = radius, c = radius;  // dimensions of the ellipsoid.
  // Check if we need to adjust the sphere to ellipsoid.
  if (dist_min_sq < (radius * radius)) {
    // Reduce other axes
    b = std::sqrt(
        (p_tangent.y() * p_tangent.y() + p_tangent.z() * p_tangent.z()) /
        (1 - p_tangent.x() * p_tangent.x() / (a * a)));
    c = b;  // Set equal b for now; but could increase.???
    // Fit the first hyperplane: x x_l + y y_l + z z_l = 1
    Eigen::Vector3d hyperplane_last =
        Eigen::Vector3d(p_tangent.x() / (a * a), p_tangent.y() / (b * b),
                        p_tangent.z() / (c * c));
    hyperplane_list.push_back(hyperplane_last);
    tangent_point_list.push_back(p_tangent);
  }

  // Increase the ellipsoid and repeat.
  bool stop = false;
  int n_max = 0;  // magic number: max 50 hyperplanes
  while ((!stop) && (n_max < 50)) {
    ++n_max;
    pcl::PointCloud<pcl::PointXYZ>* pcl_reduced(
        new pcl::PointCloud<pcl::PointXYZ>());
    // Also re-scale each dimension followed the dimentions of ellipsoid
    if (hyperplane_list.size()) {
      Eigen::Vector3d hyperplane_last;
      hyperplane_last = hyperplane_list.back();
      // Reduce point: keep points on the same side with zero origin (sign < 0)
      for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
        double sign = p->x * hyperplane_last.x() + p->y * hyperplane_last.y() +
                      p->z * hyperplane_last.z() - 1;
        const double kDSign = -0.05;  // numeric issue
        if (sign < kDSign) {
          // same side with the ellipsoid.
          pcl_reduced->push_back(*p);
        }
      }
    } else {
      pcl::copyPointCloud(*pcl_tf, *pcl_reduced);
    }

    Eigen::Vector3d p_tangent1;
    dist_min_sq = std::numeric_limits<double>::max();
    for (auto p = pcl_reduced->begin(); p != pcl_reduced->end(); ++p) {
      // Scale to get next closest point.
      pcl::PointXYZ pv;
      pv.x = p->x / a;
      pv.y = p->y / b;
      pv.z = p->z / c;
      double dist_t = pv.x * pv.x + pv.y * pv.y + pv.z * pv.z;
      if (dist_t < dist_min_sq) {
        dist_min_sq = dist_t;
        p_tangent1 << p->x, p->y, p->z;
      }
    }
    if ((pcl_reduced->size() == 0) ||
        (dist_min_sq == std::numeric_limits<double>::max())) {
      stop = true;
    } else {
      double e_ext = dist_min_sq;
      Eigen::Vector3d hyperplane_new = Eigen::Vector3d(
          p_tangent1.x() / (a * a * e_ext), p_tangent1.y() / (b * b * e_ext),
          p_tangent1.z() / (c * c * e_ext));
      hyperplane_list.push_back(hyperplane_new);
      tangent_point_list.push_back(p_tangent1);
      pcl_tf->clear();
      pcl::copyPointCloud(*pcl_reduced, *pcl_tf);
    }
  }
  if (!stop) {
    // Require too many hyperplanes
    return false;
  }

  // Find the polygon formed from intersections between the bisector plane vs.
  // all hyperplanes Not sure how to get the closed-form solution, also issue
  // with unknown voxels
  //  --> get average from uniform sampling on the y-z plane (body coordinate)
  std::vector<Eigen::Vector3d> feasible_samples;
  for (double dy = -kDy; dy < kDy; dy += 0.1) {
    for (double dz = -kDz; dz < kDz; dz += 0.1) {
      Eigen::Vector3d p(radius, dy, dz);
      // check if this is inside all hyperplanes.
      const double kDSign = -0.05;  // numeric issue
      double sign;
      int i = 0;
      for (i = 0; i < hyperplane_list.size(); ++i) {
        sign = p.x() * hyperplane_list[i].x() + p.y() * hyperplane_list[i].y() +
               p.z() * hyperplane_list[i].z() - 1;
        if (sign > kDSign) break;
      }
      if (i == hyperplane_list.size()) {
        feasible_samples.push_back(p);
      }
    }
  }

  for (int i = 0; i < hyperplane_list.size(); ++i) {
    tangent_point_list[i] =
        tf_W2S * tangent_point_list[i];  // convert back to world
    Eigen::Matrix4d tf_inv_T = tf_W2S.matrix().inverse().transpose();
    Eigen::Vector4d v_t;
    v_t = tf_inv_T * Eigen::Vector4d(hyperplane_list[i].x(),
                                     hyperplane_list[i].y(),
                                     hyperplane_list[i].z(), -1.0);
    v_t = v_t / (-v_t[3]);
    hyperplane_list[i] << v_t.x(), v_t.y(), v_t.z();
  }

  p1_mod << 0.0, 0.0, 0.0;
  int feasible_count = 0;
  for (int i = 0; i < feasible_samples.size(); ++i) {
    feasible_samples[i] =
        tf_W2S * feasible_samples[i];  // convert back to world
    // check if this is free voxel to deal with occluded area.
    if (map_manager_->getVoxelStatus(feasible_samples[i]) ==
        MapManager::VoxelStatus::kFree) {
      feasible_corridor_pcl_->push_back(pcl::PointXYZ(feasible_samples[i].x(),
                                                      feasible_samples[i].y(),
                                                      feasible_samples[i].z()));
      p1_mod = p1_mod + feasible_samples[i];
      ++feasible_count;
    }
  }

  if (feasible_count) {
    p1_mod = p1_mod / feasible_count;
  } else {
    return false;
  }

  return true;
}

bool GraphManager::improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                          std::vector<geometry_msgs::Pose>& path_mod,
                          bool relaxed) {
  // Few heuristics to improve the path.
  // a) Shorten path by reducing intermidiate nodes. (be careful with turning
  // cases) Shorten/reduce some very short paths to prevent small motion and
  // sudden change in angles. b) Adjust nodes to its neighbors to improve safety
  // c) Ignore leaf node of the path to prevent the robot to come too close the
  // obstacle

  if (path_orig.empty()) return false;

  // Interpolate path
  std::vector<geometry_msgs::Pose> path_orig_interp = path_orig;
  double kInterpolationDistance =
      planning_params_.path_interpolation_distance * 2.0;
  std::vector<geometry_msgs::Pose> interp_path_orig;
  if (Trajectory::interpolatePath(path_orig, kInterpolationDistance,
                                  interp_path_orig)) {
    path_orig_interp = interp_path_orig;
  }

  // Feature a) Remove short intermidiate vertices.
  std::vector<geometry_msgs::Pose> path_mod1 = path_orig_interp;

  const double kSegmentLenMin = 0.5;
  bool cont_refine = true;
  while (cont_refine) {
    cont_refine = false;
    if (path_mod1.size() > 2) {
      for (int i = 0; i < (path_mod1.size() - 2); ++i) {
        Eigen::Vector3d p_start(path_mod1[i].position.x,
                                path_mod1[i].position.y,
                                path_mod1[i].position.z);
        Eigen::Vector3d p_int(path_mod1[i + 1].position.x,
                              path_mod1[i + 1].position.y,
                              path_mod1[i + 1].position.z);
        Eigen::Vector3d p_end(path_mod1[i + 2].position.x,
                              path_mod1[i + 2].position.y,
                              path_mod1[i + 2].position.z);
        Eigen::Vector3d segment = p_int - p_start;
        double segment_len = segment.norm();
        // ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Segment length %f.", segment_len);
        if (segment_len < kSegmentLenMin) {
          if ((MapManager::VoxelStatus::kFree ==
               map_manager_->getPathStatus(p_start, p_end, robot_box_size_,
                                           false)) &&
              (!planning_params_.geofence_checking_enable ||
               (GeofenceManager::CoordinateStatus::kOK ==
                geofence_manager_->getPathStatus(
                    Eigen::Vector2d(p_start[0], p_start[1]),
                    Eigen::Vector2d(p_end[0], p_end[1]),
                    Eigen::Vector2d(robot_box_size_[0],
                                    robot_box_size_[1]))))) {
            // ignore the intermidiate nore, combine the first to the last node.
            ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Combine nodes to remove short segments.");
            path_mod1.erase(path_mod1.begin() + i + 1);
            cont_refine = true;
            break;
          }
        }
      }
    } else {
      return false;
    }
  }

  // Implement (b) first: form a safe corridor along each path from cutting
  // hyperplanes.
  feasible_corridor_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  geometry_msgs::Pose pose0;
  pose0.position.x = path_mod1[0].position.x;
  pose0.position.y = path_mod1[0].position.y;
  pose0.position.z = path_mod1[0].position.z;
  pose0.orientation.x = path_mod1[0].orientation.x;
  pose0.orientation.y = path_mod1[0].orientation.y;
  pose0.orientation.z = path_mod1[0].orientation.z;
  pose0.orientation.w = path_mod1[0].orientation.w;
  path_mod.push_back(pose0);
  bool mod_success = true;

  for (int i = 1; i < path_mod1.size(); ++i) {
    Eigen::Vector3d p0(path_mod1[i - 1].position.x, path_mod1[i - 1].position.y,
                       path_mod1[i - 1].position.z);
    Eigen::Vector3d p0_mod(path_mod[i - 1].position.x,
                           path_mod[i - 1].position.y,
                           path_mod[i - 1].position.z);
    Eigen::Vector3d p1(path_mod1[i].position.x, path_mod1[i].position.y,
                       path_mod1[i].position.z);
    Eigen::Vector3d p1_parallel = p0_mod + p1 - p0;

    Eigen::Vector3d p2;
    bool do_check_p2 = false;
    if (i < path_mod1.size() - 1) {
      do_check_p2 = true;
      p2 = Eigen::Vector3d(path_mod1[i + 1].position.x,
                           path_mod1[i + 1].position.y,
                           path_mod1[i + 1].position.z);
    }

    Eigen::Vector3d p1_target;
    bool seg_free = true;

    bool e1_admissible = false;
    bool e2_admissible = false;
    if (robot_params_.type == RobotType::kAerialRobot) {
      MapManager::VoxelStatus vs1 = map_manager_->getPathStatus(
          p0_mod, p1_parallel, robot_box_size_, true);
      if (vs1 == MapManager::VoxelStatus::kFree) e1_admissible = true;
      MapManager::VoxelStatus vs2 =
          map_manager_->getPathStatus(p0_mod, p1, robot_box_size_, true);
      if (vs2 == MapManager::VoxelStatus::kFree) e2_admissible = true;
    } 
    if (e1_admissible) {
      p1_target = p1_parallel;
    } else if (e2_admissible) {
      p1_target = p1;
    } else {
      ROS_WARN("Segment not free \n");
      seg_free = false;
    }

    Eigen::Vector3d p1_mod;
    geometry_msgs::Pose pose;

    Eigen::Vector3d p_center;
    p_center = (p0_mod + p1_target) / 2.0;
    Eigen::Vector3d p_dir;
    p_dir = (p1 - p0);
    double radius = p_dir.norm() / 2.0;
    // add a local bounding box
    Eigen::Vector3d safety_extension;
    safety_extension = robot_params_.safety_extension;
    if (relaxed)
      safety_extension(1) *= planning_params_.relaxed_corridor_multiplier;
    Eigen::Vector3d local_bbx(2 * (radius + safety_extension[0]),
                              2 * safety_extension[1], 2 * safety_extension[2]);
    std::vector<Eigen::Vector3d> occupied_voxels;
    std::vector<Eigen::Vector3d> free_voxels;
    map_manager_->extractLocalMapAlongAxis(p_center, p_dir, local_bbx,
                                           occupied_voxels, free_voxels);

    pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (auto& v : occupied_voxels) {
      obstacle_pcl->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
    }
    
    bool modification_successful =
        (modifyPath(obstacle_pcl, p0_mod, p1_target, p1_mod));
    if (seg_free && modification_successful) {
      bool e1_admissible = false;
      bool e2_admissible = false;
      if (robot_params_.type == RobotType::kAerialRobot) {
        MapManager::VoxelStatus vs1 =
            map_manager_->getPathStatus(p0_mod, p1_mod, robot_box_size_, true);
        if (vs1 == MapManager::VoxelStatus::kFree) e1_admissible = true;
        MapManager::VoxelStatus vs2 =
            map_manager_->getPathStatus(p1_mod, p2, robot_box_size_, true);
        if (vs2 == MapManager::VoxelStatus::kFree) e2_admissible = true;
      } 

      if (!(e1_admissible) ||
          (planning_params_.geofence_checking_enable &&
           (GeofenceManager::CoordinateStatus::kViolated ==
            geofence_manager_->getPathStatus(
                Eigen::Vector2d(p0_mod[0], p0_mod[1]),
                Eigen::Vector2d(p1_mod[0], p1_mod[1]),
                Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) ||
          (do_check_p2 && (!(e2_admissible) ||
                           (planning_params_.geofence_checking_enable &&
                            (GeofenceManager::CoordinateStatus::kViolated ==
                             geofence_manager_->getPathStatus(
                                 Eigen::Vector2d(p1_mod[0], p1_mod[1]),
                                 Eigen::Vector2d(p2[0], p2[1]),
                                 Eigen::Vector2d(robot_box_size_[0],
                                                 robot_box_size_[1]))))))) {
        p1_mod = p1;
        mod_success = false;
        ROS_WARN("Newly modified path is not collision-free.");
        // break; // break to save time @recheck
      }
    } else {
      p1_mod = p1;
      // mod_success = false;
      // break; // break to save time @recheck
    }
    pose.position.x = p1_mod.x();
    pose.position.y = p1_mod.y();
    pose.position.z = p1_mod.z();
    path_mod.push_back(pose);
  }

  // Correct the heading angle tangent with the moving direction again.
  // Re-Assign the first heading
  path_mod[0].orientation.x = path_orig_interp[0].orientation.x;
  path_mod[0].orientation.y = path_orig_interp[0].orientation.y;
  path_mod[0].orientation.z = path_orig_interp[0].orientation.z;
  path_mod[0].orientation.w = path_orig_interp[0].orientation.w;
  // if ((mod_success) && (planning_params_.yaw_tangent_correction)) {
  //   for (int i = 1; i < path_mod.size(); ++i) {
  //     Eigen::Vector3d vec(path_mod[i].position.x - path_mod[i - 1].position.x,
  //                         path_mod[i].position.y - path_mod[i - 1].position.y,
  //                         path_mod[i].position.z - path_mod[i - 1].position.z);
  //     if (planning_params_.planning_backward) vec = -vec;
  //     double yawhalf = 0.5 * std::atan2(vec[1], vec[0]);
  //     path_mod[i].orientation.x = 0.0;
  //     path_mod[i].orientation.y = 0.0;
  //     path_mod[i].orientation.z = sin(yawhalf);
  //     path_mod[i].orientation.w = cos(yawhalf);
  //   }
  // }

  // visualization_->visualizePCL(feasible_corridor_pcl_.get());
  return mod_success;
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

  for (std::vector<Vertex *>::iterator it_vert = raw_path.begin(); it_vert != std::prev(raw_path.end(), 0); it_vert++)
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

      if (edge_length < 1.5 * max_edge_length_ and MapManager::VoxelStatus::kFree == vn)
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

// Calculate relative Angle of a point wrt robot's current heading vector

double GraphManager::calculateAngle(Eigen::Vector3d fpoint)
{
    return atan2((fpoint.y() - robotState_[1]), (fpoint.x() - robotState_[0])) * (180 / M_PI) - yaw_;
}

bool GraphManager::isLocal(Eigen::Vector3d point)
{

    double angle = calculateAngle(point);
    double distance = calculateDistance(eigenToState(point), robotState_);

    if (((angle < m_local_fov) && (angle > -m_local_fov)) && distance < (bounding_box_size_ + 2))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void GraphManager::updateGraph(const ros::TimerEvent &event)
{
  
  // Can off load some repetitive stuff here : executed  ehile robot is navigated so not added to time complexity
  
  // GraphManager::updateSparseGraph();

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

  bool planner_status = true;
  
  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  
  switch (reached_end_of_path_) {
    case 1:

      exploration_path_.poses.clear();
      if (GraphManager::buildLocalGraph()) {
        
        std::vector<std::vector<Vertex *>> dijkstra_paths;
        dijkstra_paths.clear();

        if (GraphManager::extractShortestPaths(sub_graph_, sub_graph_rep_, local_root_vertex, dijkstra_paths)) {
          
          visualizeLocalGraph();
          GraphManager::insertBestPathsToSparseGraph(dijkstra_paths);
          ROS_INFO("Successfully inserted best paths \n");
          if (GraphManager::planLocalPath(dijkstra_paths)) {

            ROS_INFO("Successfully planned local path \n");
            
            if (GraphManager::setPath()) {

              ROS_INFO("local path set success! \n");
              res.path = exploration_path_;
              ++planning_iteration_;
              break;
            } else {
              ROS_WARN("Local path setting -> Failure ! \n");
            }
          } else {
            
            ROS_WARN("Local planner failure ! Attempting global repositioning \n");
        
            std::vector<std::vector<Vertex *>> dijkstra_paths;
            dijkstra_paths.clear();

            if (GraphManager::extractShortestPaths(sparse_graph_, sparse_graph_rep_, global_root_vertex, dijkstra_paths)) {

              if (GraphManager::planGlobalPath(dijkstra_paths)) {

                ROS_INFO ("Successfully planner global paths \n");

                if (GraphManager::setPath()) {
                  ROS_INFO("Global path set success !\n");
                  res.path = exploration_path_;
                  ++planning_iteration_;
                  break;
                } else {
                  ROS_WARN("Global planner failure \n");
                  break;
                }
              }
            }
          }

        } else {
          ROS_WARN("Could not extract shortest paths on sub graph \n");
          
          if (GraphManager::expandSparseGraph()) {

            ROS_INFO("Expanded sparse graph for global planning \n");
            std::vector<std::vector<Vertex *>> dijkstra_paths;
            dijkstra_paths.clear();

            if (GraphManager::extractShortestPaths(sparse_graph_, sparse_graph_rep_, global_root_vertex, dijkstra_paths)) {

              if (GraphManager::planGlobalPath(dijkstra_paths)) {

                ROS_INFO ("Successfully planner global paths \n");

                if (GraphManager::setPath()) {
                  ROS_INFO("Global path set success !\n");
                  res.path = exploration_path_;
                  ++planning_iteration_;
                  break;
                } else {
                  ROS_WARN("Global planner failure \n");
                  break;
                }
              }
            }
          }
        } 
      } 
      

    case 0:
      planner_status = GraphManager::setPath();
      res.path = exploration_path_;
      // ROS_INFO("Executing path segment \n");
  }

  high_resolution_clock::time_point stop_total = high_resolution_clock::now();

  auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  if (duration_total.count() > 0) {

  ROS_INFO_STREAM("\033[;34m[Planning iter -> " << planning_iteration_ << "] \t Exec time :" << duration_total.count() << " ms \033[0m\n");
  }

  return planner_status;

}

// Visualization of navigation graphs

void GraphManager::visualizeTree (std::vector<node *> RRT_TREE) {

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker RRT_points, RRT_line_list;
  RRT_points.header.frame_id = RRT_line_list.header.frame_id = "world";
  RRT_points.ns = "nodes";
  RRT_line_list.ns = "edges";
  RRT_points.action = visualization_msgs::Marker::ADD;
  RRT_points.pose.orientation.w = 1.0;
  RRT_points.id = 0;
  RRT_line_list.id = 1;
  RRT_points.type = visualization_msgs::Marker::POINTS;
  RRT_line_list.type = visualization_msgs::Marker::LINE_LIST;
  RRT_points.scale.x = 0.2;
  RRT_points.scale.y = 0.2;
  RRT_line_list.scale.x = 0.1;
  RRT_points.color.g = 1.0f;
  RRT_points.color.a = 1.0;
  RRT_line_list.color.b = 1.0;
  RRT_line_list.color.a = 1.0;
  std::vector<node*>::iterator it_comeon_visualizer;


  for(it_comeon_visualizer = RRT_TREE.begin(); it_comeon_visualizer != RRT_TREE.end(); it_comeon_visualizer++){
    geometry_msgs::Point p;
    p.x = (*it_comeon_visualizer)->point->x();
    p.y = (*it_comeon_visualizer)->point->y();
    p.z = (*it_comeon_visualizer)->point->z();
    RRT_points.points.push_back(p);
    if((*it_comeon_visualizer)->myParent != nullptr){
      RRT_line_list.points.push_back(p);
      p.x = (*it_comeon_visualizer)->myParent->point->x();
      p.y = (*it_comeon_visualizer)->myParent->point->y();
      p.z = (*it_comeon_visualizer)->myParent->point->z();

      RRT_line_list.points.push_back(p);
    }
  }
  marker_array.markers.push_back(RRT_points);
  marker_array.markers.push_back(RRT_line_list);
  tree_Vis_Pub_.publish(marker_array);

}




void GraphManager::visualizePathWaysGraph()
{

  std::shared_ptr<Graph> g = path_ways_graph_->graph_;
  std::unordered_map<int, Vertex *> &v_map = path_ways_graph_->vertices_map_;
  
  if (path_ways_graph_->getNumVertices() == 0) {
    return;
  }

  visualization_msgs::MarkerArray marker_array;

  std_msgs::ColorRGBA vertex_color;
  std_msgs::ColorRGBA frontier_color;
  vertex_color.r = 125.0 / 255.0;
  vertex_color.g = 42.0 / 255.0;
  vertex_color.b = 104.0 / 255.0;
  vertex_color.a = 1.0;
  frontier_color.r = 250 / 255.0;
  frontier_color.g = 0;
  frontier_color.b = 0;
  frontier_color.a = 1.0;

  
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = "world";
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 90.0 / 255.0;
  edge_marker.color.g = 80.0 / 255.0;
  edge_marker.color.b = 100.0;
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
  vertex_marker.scale.x = 0.4;
  vertex_marker.scale.y = 0.4;
  vertex_marker.scale.z = 0.4;
  vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    int id = g->getVertexProperty(it);
    if (v_map[id]->is_frontier and v_map[id]->is_cluster_centroid)
    {

      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      vertex_marker.colors.push_back(frontier_color);
      vertex_marker.points.push_back(p1);
    } else if (v_map[id]->is_cluster_centroid) {

      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      vertex_marker.colors.push_back(vertex_color);
      vertex_marker.points.push_back(p1);
    }

  }
  marker_array.markers.push_back(vertex_marker);

  path_Ways_Graph_Vis_Pub_.publish(marker_array);
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
    if (v_map[id]->vol_gain.gain > maxZ)
    // if (v_map[id]->state[2] > maxZ)
    {
      // maxZ = v_map[id]->state[2];
      maxZ = v_map[id]->vol_gain.gain;
    }

    if (v_map[id]->vol_gain.gain < minZ)
    // if (v_map[id]->state[2] < minZ)
    {
      // minZ = v_map[id]->state[2];
      minZ = v_map[id]->vol_gain.gain;
    }
  }

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    double gain = v_map[id]->vol_gain.gain;
    // double h = (1.0 - std::min(std::max((p1.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * 0.8;
    double h = (1.0 - std::min(std::max((gain - minZ) / (maxZ - minZ), 0.0), 1.0)) * 0.8;
    vertex_marker.colors.push_back(GraphManager::heightMapColor(h));
    vertex_marker.points.push_back(p1);
    // if (v_map[id]->is_frontier)
    // if (v_map[id]->is_leaf_vertex)
    // if (v_map[id]->vol_gain.is_frontier and v_map[id]->is_cluster_centroid)
    if (v_map[id]->vol_gain.is_frontier and v_map[id]->is_cluster_centroid)
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
  edge_marker.color.r = 0.0 / 255.0;
  edge_marker.color.g = 200.0 / 255.0;
  edge_marker.color.b = 0.0 / 255.0;
  edge_marker.color.a = 0.8;
  // edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = "world";
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.2;
  vertex_marker.scale.y = 0.2;
  vertex_marker.scale.z = 0.2;
  vertex_marker.color.r = 0.0 / 255.0;
  vertex_marker.color.g = 200 / 255.0;
  vertex_marker.color.b = 0.0 / 255.0;
  vertex_marker.color.a = 0.8;
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
  edge_marker.color.r = 90.0 / 255.0;
  edge_marker.color.g = 80.0 / 255.0;
  edge_marker.color.b = 100.0;
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
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.frame_locked = false;
  vertex_marker.color.r = 125.0 / 255.0;
  vertex_marker.color.g = 42.0 / 255.0;
  vertex_marker.color.b = 104.0 / 255.0;
  vertex_marker.color.a = 1.0;
  
  visualization_msgs::Marker frontier_vertex_marker;
  frontier_vertex_marker.header.stamp = ros::Time::now();
  frontier_vertex_marker.header.seq = 0;
  frontier_vertex_marker.header.frame_id = "world";
  frontier_vertex_marker.id = 0;
  frontier_vertex_marker.ns = "frontier";
  frontier_vertex_marker.action = visualization_msgs::Marker::ADD;
  frontier_vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
  frontier_vertex_marker.scale.x = 0.3;
  frontier_vertex_marker.scale.y = 0.3;
  frontier_vertex_marker.scale.z = 0.3;
  frontier_vertex_marker.color.r = 0.0 / 255.0;
  frontier_vertex_marker.color.g = 0.0 / 255.0;
  frontier_vertex_marker.color.b = 200.0 / 255.0;
  frontier_vertex_marker.color.a = 0.8;
  frontier_vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
  Graph::GraphType::vertex_iterator>
  vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it)
  {
    int id = g->getVertexProperty(it);
    if (v_map[id]->vol_gain.is_frontier and v_map[id]->is_cluster_centroid)
    {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];

      frontier_vertex_marker.points.push_back(p1);
    } else {
   
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];

      vertex_marker.points.push_back(p1);

    }

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

void GraphManager::segmentPath(nav_msgs::Path path,
                               nav_msgs::Path &path_seg)
{
  float dt_ = 0.5;
  path_seg.poses.clear();
  double v_max_ = 0.5;
  double yaw_rate_max_ = 0.5;
  if (path.poses.size() == 0)
    return;

  for (int i = 0; i < (path.poses.size() - 1); ++i)
  {
    Eigen::Vector3d start(path.poses[i].pose.position.x, path.poses[i].pose.position.y,
                          path.poses[i].pose.position.z);
    Eigen::Vector3d end(path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y,
                        path.poses[i + 1].pose.position.z);
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
