#include "graph_management.h"

using namespace std::chrono;

GraphManager::GraphManager(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
        map_manager)
    : nh_(nh),
      nh_private_(nh_private),
      map_manager_(map_manager)

{
  GraphManager::initializeAttributes();
};

void GraphManager::reset() {
  // Re-initialize data structs.

  stat_.reset(new SampleStatistic());
  stat_chrono_.reset(new SampleStatistic());
  geofence_manager_.reset(new GeofenceManager());
  random_sampler_.reset();
  random_sampler_manual_mode_.reset();
  random_sampler_graph_expansion_.reset();

  exploration_dir_pose.header.frame_id = world_frame_id;
  exploration_dir_pose.header.stamp = ros::Time::now();
  exploration_dir_pose.pose.position.x = robotState_[0];
  exploration_dir_pose.pose.position.y = robotState_[1];
  exploration_dir_pose.pose.position.z = robotState_[2];
  exploration_dir_pose.pose.orientation.x = 0;
  exploration_dir_pose.pose.orientation.y = 0;
  exploration_dir_pose.pose.orientation.z = 0;
  exploration_dir_pose.pose.orientation.w = 1;

  // First check if this position is free to go.
  MapManager::VoxelStatus voxel_state = map_manager_->getBoxStatus(
      Eigen::Vector3d(robotState_[0], robotState_[1], robotState_[2]) +
          robot_params_.center_offset,
      robot_box_size_, true);
  if (MapManager::VoxelStatus::kFree != voxel_state) {
    switch (voxel_state) {
      case MapManager::VoxelStatus::kFree:
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG,
                      "Current box is Free.");
        break;
      case MapManager::VoxelStatus::kOccupied:
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG,
                      "Current box contains Occupied voxels.");
        break;
      case MapManager::VoxelStatus::kUnknown:
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG,
                      "Current box contains Unknown voxels.");
        break;
    }
    // Assume that even it is not fully free, but safe to clear these voxels.
    ROS_WARN_COND(
        global_verbosity >= Verbosity::WARN,
        "Starting position is not clear--> clear space around the robot.");
    map_manager_->augmentFreeBox(
        Eigen::Vector3d(robotState_[0], robotState_[1], robotState_[2]) +
            robot_params_.center_offset,
        robot_box_size_);
  }

  // Set robot radius when single point collision checking is used
  map_manager_->setRobotRadius(robot_box_size_.norm() / 2.0);

  // Clear free space before planning.
  if (planning_params_.free_frustum_before_planning) {
    map_manager_->augmentFreeFrustum();
  }

  std::vector<double> empty_vec;
  for (int i = 0; i < planning_params_.num_vertices_max; ++i) {
    empty_vec.push_back(0.0);
  }
  edge_inclinations_.clear();
  for (int i = 0; i < planning_params_.num_vertices_max; ++i) {
    edge_inclinations_.push_back(empty_vec);
  }

  GraphManager::resetPoseGraph();
  GraphManager::resetLocalGraph();
  GraphManager::resetSparseGraph();
  GraphManager::resetGlobalGraph();
  GraphManager::resetPathWaysGraph();
}

void GraphManager::initializeAttributes() {
  visualization_server_ = new VisualizationServer(nh_, nh_private_);

  stat_.reset(new SampleStatistic());
  stat_chrono_.reset(new SampleStatistic());
  geofence_manager_.reset(new GeofenceManager());
  random_sampler_.reset();
  random_sampler_manual_mode_.reset();
  random_sampler_graph_expansion_.reset();
  stat_->init(robotState_);
}

void GraphManager::loadParams() {
  std::string ns = ros::this_node::getName();
  sensor_params_.loadParams(ns + "/SensorParams");
  sensor_name = sensor_params_.sensor_list[0];
  robot_params_.loadParams(ns + "/RobotParams");
  global_space_params_.loadParams(ns + "/BoundedSpaceParams/Global");
  local_space_params_.loadParams(ns + "/BoundedSpaceParams/Local");
  manual_mode_params_.loadParams(ns + "/BoundedSpaceParams/LocalSearch");
  graph_expansion_global_params_.loadParams(ns +
                                            "/GraphExpansionParams/Global");
  graph_expansion_local_params_.loadParams(ns + "/GraphExpansionParams/Local");
  planning_params_.loadParams(ns + "/PlanningParams");

  geofence_manager_->loadParams(ns + "/GeofenceParams");
  // robot_state_hist_.reset(new RobotStateHistory());

  random_sampler_.loadParams(ns + "/RandomSamplerParams/SamplerForExploration");
  random_sampler_manual_mode_.loadParams(
      ns + "/RandomSamplerParams/SamplerForExploration");
  random_sampler_graph_expansion_.loadParams(
      ns + "/RandomSamplerParams/SamplerForExploration");

  random_sampler_.setParams(global_space_params_, local_space_params_);
  random_sampler_manual_mode_.setParams(global_space_params_,
                                        manual_mode_params_);
  random_sampler_graph_expansion_.setParams(graph_expansion_global_params_,
                                            graph_expansion_local_params_);
  map_manager_->setRobotRadius(robot_box_size_.norm() / 2.0);
}

void GraphManager::setSharedParams(
    const RobotParams& robot_params,
    const BoundedSpaceParams& global_space_params,
    const BoundedSpaceParams& local_space_params,
    const BoundedSpaceParams& manual_mode_params) {
  robot_params_ = robot_params;
  robot_params_.getPlanningSize(robot_box_size_);

  global_space_params_ = global_space_params;
  local_space_params_ = local_space_params;
  manual_mode_params_ = manual_mode_params;
}

void GraphManager::resetPoseGraph() {
  if (pose_graph_ != NULL) {
    pose_graph_->reset();
  }
  pose_graph_.reset(new GraphUtils());
  pose_graph_rep_.reset();
}

void GraphManager::resetLocalGraph() {
  if (local_graph_ != NULL) {
    local_graph_->reset();
  }
  local_graph_.reset(new GraphUtils());
  local_graph_rep_.reset();
  ++subgraph_ind_;
  local_graph_->subgraph_ind_ = subgraph_ind_;
}

void GraphManager::resetProjectedGraph() {
  if (projected_graph_ != NULL) {
    projected_graph_->reset();
  }
  projected_graph_.reset(new GraphUtils());
}

void GraphManager::resetGlobalGraph() {
  StateVec home_state;
  // Adding home vertex at initialization phase for simpler RTB planning
  home_state[0] = robotState_[0];
  home_state[1] = robotState_[1];
  home_state[2] = robotState_[2] + 1;
  home_state[2] = robotState_[3];

  if (global_graph_ != NULL) {
    global_graph_->reset();
  }
  global_graph_.reset(new GraphUtils());
  global_graph_rep_.reset();
}

void GraphManager::setState(StateVec& state) {
  if (!odometry_ready) {
    // First time receive the pose/odometry for planning purpose.
    // Reset the voxblox map manager
    ROS_WARN("Received the first odometry, resetting the map manager");
    map_manager_->resetMap();
    prev_state = state;
  }
  robotState_ = state;
  if ((prev_state.head(3) - robotState_.head(3)).norm() > 1) {
    Vertex* prev_vertex =
        new Vertex(pose_graph_->generateVertexID(), prev_state);
    Vertex* vertex = new Vertex(pose_graph_->generateVertexID(), robotState_);
    pose_graph_->addVertex(prev_vertex);
    pose_graph_->addVertex(vertex);
    double weight = (prev_state.head(3) - robotState_.head(3)).norm();
    pose_graph_->addEdge(prev_vertex, vertex, weight);
    prev_state = robotState_;
    visualization_server_->visualizePoseGraph(pose_graph_);
    GraphManager::updateGlobalGraph();
  }
  odometry_ready = true;
}

void GraphManager::resetPathWaysGraph() {
  if (path_ways_graph_ != NULL) {
    path_ways_graph_->reset();
  }
  path_ways_graph_.reset(new GraphUtils());
  path_ways_graph_rep_.reset();
}

void GraphManager::resetSparseGraph() {
  if (sparse_graph_ != NULL) {
    sparse_graph_->reset();
  }
  sparse_graph_.reset(new GraphUtils());
  sparse_graph_rep_.reset();
}

void GraphManager::setGlobalFrame(std::string frame_id) {
  if (!frame_id.empty()) {
    visualization_server_->setGlobalFrame(frame_id);
    world_frame_id = frame_id;
  }
}

StateVec GraphManager::eigenToState(Eigen::Vector3d v) {
  StateVec s(v[0], v[1], v[2], 0);
  return s;
}

double GraphManager::getPathLength(std::vector<Vertex*> path) {
  double l = 0;

  for (int i = 0; i < path.size() - 1; ++i) {
    l = l + (path[i]->state.head(2) - path[i + 1]->state.head(3)).norm();
  }
  return l;
}

Eigen::Vector3d GraphManager::quaternion_to_rpy(Eigen::Quaterniond q) {
  const double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  const double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
  const double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

  return Eigen::Vector3d{roll, pitch, yaw};
}

Eigen::Quaternion<float> GraphManager::Euler2Quaternion(Eigen::Vector3d v) {
  float roll = v.x();
  float pitch = v.y();
  float yaw = v.z();

  Eigen::Quaternion<float> q;

  q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  return q;
}

void GraphManager::computeVolumetricGainRayCasting(StateVec& state,
                                                   VolumetricGain& vgain) {
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
      vgain.unknown_voxels.push_back(voxel);
    } else if (vs == MapManager::VoxelStatus::kFree) {
      ++num_free_voxels;
      vgain.free_voxels.push_back(voxel);
    } else if (vs == MapManager::VoxelStatus::kOccupied) {
      ++num_occupied_voxels;
      vgain.occupied_voxels.push_back(voxel);
    } else {
      ROS_ERROR("Unsupported voxel type.");
    }
  }
  gain_log.push_back(std::make_tuple(num_unknown_voxels, num_free_voxels,
                                     num_occupied_voxels,
                                     num_unknown_surf_voxels));

  // Check if it is a potential frontier.
  // double unknown_voxels_scaled = (num_unknown_voxels - 2*num_occupied_voxels)
  // * map_manager_->getResolution();
  double unknown_voxels_scaled =
      num_unknown_voxels * map_manager_->getResolution();
  if (sensor_params_.sensor[sensor_name].isFrontier(unknown_voxels_scaled,
                                                    vgain.unknown_percentage)) {
    vgain.is_frontier = true;  // Event E2
  }

  // Return gain values.
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    // vgain.gain += (num_unknown_voxels * 60 - num_occupied_voxels * 40);
    vgain.gain += (num_unknown_voxels * 60);
  }
}

bool GraphManager::sampleState(StateVec root_state, StateVec& vertex_state) {
  StateVec state;
  bool hanging = false;
  bool found = false;

  int while_thres = 1000;  // magic number
  BoundedSpaceParams reduced_global_space = global_space_params_;
  reduced_global_space.min_val += 0.5 * robot_box_size_;
  reduced_global_space.max_val -= 0.5 * robot_box_size_;
  while (!found && while_thres--) {
    hanging = false;
    random_sampler_.generate(root_state, state);
    // Very fast check if the sampled point is inside the planning space.
    // This helps eliminate quickly points outside the sampling space.
    Eigen::Vector3d sample = state.head(3);
    if (!reduced_global_space.isInsideSpace(sample)) continue;

    // Check if in geofence areas.
    if ((planning_params_.geofence_checking_enable) &&
        (GeofenceManager::CoordinateStatus::kViolated ==
         geofence_manager_->getBoxStatus(
             Eigen::Vector2d(state[0] + robot_params_.center_offset[0],
                             state[1] + robot_params_.center_offset[1]),
             Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))))
      continue;

    if (robot_params_.type == RobotType::kGroundRobot) {
      Eigen::Vector3d sample;
      sample = Eigen::Vector3d(state[0], state[1], state[2]) +
               robot_params_.center_offset;
      MapManager::VoxelStatus vs;
      double ground_dist = projectSample(sample, vs);
      if (ground_dist < 0.0) continue;
      if (vs == MapManager::VoxelStatus::kUnknown) {
        hanging = true;
      }

      sample[2] -= (ground_dist - planning_params_.max_ground_height);
      state[0] = sample[0];
      state[1] = sample[1];
      state[2] -= (ground_dist - planning_params_.max_ground_height);
    }

    // Check if surrounding area is free.
    if (MapManager::VoxelStatus::kFree ==
        map_manager_->getBoxStatus(
            Eigen::Vector3d(state[0], state[1], state[2]) +
                robot_params_.center_offset,
            robot_box_size_, true)) {
      random_sampler_.pushSample(state, true);  // for debug purpose.
      found = true;
    } else {
      // stat_->num_vertices_fail++;
      random_sampler_.pushSample(state, false);
    }
  }
  vertex_state = state;
  // vertex.is_hanging = hanging;
  return found;
}

bool GraphManager::sampleStateToExpandGraph(StateVec root_state,
                                            StateVec& vertex_state) {
  StateVec state;
  bool hanging = false;
  bool found = false;

  int while_thres = 1000;  // magic number
  BoundedSpaceParams reduced_global_space = global_space_params_;
  while (!found && while_thres--) {
    hanging = false;
    random_sampler_graph_expansion_.generate(root_state, state);
    // Very fast check if the sampled point is inside the planning space.
    // This helps eliminate quickly points outside the sampling space.
    Eigen::Vector3d sample = state.head(3);
    if (!reduced_global_space.isInsideSpace(sample)) continue;

    // Check if in geofence areas.
    if ((planning_params_.geofence_checking_enable) &&
        (GeofenceManager::CoordinateStatus::kViolated ==
         geofence_manager_->getBoxStatus(
             Eigen::Vector2d(state[0] + robot_params_.center_offset[0],
                             state[1] + robot_params_.center_offset[1]),
             Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))))
      continue;

    if (robot_params_.type == RobotType::kGroundRobot) {
      Eigen::Vector3d sample;
      sample = Eigen::Vector3d(state[0], state[1], state[2]) +
               robot_params_.center_offset;
      MapManager::VoxelStatus vs;
      double ground_dist = projectSample(sample, vs);
      if (ground_dist < 0.0) continue;
      if (vs == MapManager::VoxelStatus::kUnknown) {
        hanging = true;
      }

      sample[2] -= (ground_dist - planning_params_.max_ground_height);
      state[0] = sample[0];
      state[1] = sample[1];
      state[2] -= (ground_dist - planning_params_.max_ground_height);
      // state[2] = sample[2];
    }

    // Check if surrounding area is free.
    if (MapManager::VoxelStatus::kFree ==
        map_manager_->getBoxStatus(
            Eigen::Vector3d(state[0], state[1], state[2]) +
                robot_params_.center_offset,
            robot_box_size_, true)) {
      random_sampler_graph_expansion_.pushSample(state,
                                                 true);  // for debug purpose.
      found = true;
    } else {
      // stat_->num_vertices_fail++;
      random_sampler_graph_expansion_.pushSample(state, false);
    }
  }
  vertex_state = state;
  // vertex.is_hanging = hanging;
  return found;
}

bool GraphManager::sampleVertex(Vertex& vertex, bool manual_mode) {
  StateVec state;
  bool hanging = false;
  bool found = false;

  int while_thres = 1000;  // magic number
  BoundedSpaceParams reduced_global_space = global_space_params_;
  reduced_global_space.min_val += 0.5 * robot_box_size_;
  reduced_global_space.max_val -= 0.5 * robot_box_size_;
  while (!found && while_thres--) {
    hanging = false;
    if (manual_mode) {
      random_sampler_manual_mode_.generate(robotState_, state);
    } else {
      random_sampler_.generate(robotState_, state);
    }
    // Very fast check if the sampled point is inside the planning space.
    // This helps eliminate quickly points outside the sampling space.
    Eigen::Vector3d sample = state.head(3);
    if (!reduced_global_space.isInsideSpace(sample)) continue;

    // Check if in geofence areas.
    if ((planning_params_.geofence_checking_enable) &&
        (GeofenceManager::CoordinateStatus::kViolated ==
         geofence_manager_->getBoxStatus(
             Eigen::Vector2d(state[0] + robot_params_.center_offset[0],
                             state[1] + robot_params_.center_offset[1]),
             Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))))
      continue;

    if (robot_params_.type == RobotType::kGroundRobot) {
      Eigen::Vector3d sample;
      sample = Eigen::Vector3d(state[0], state[1], state[2]) +
               robot_params_.center_offset;
      MapManager::VoxelStatus vs;
      double ground_dist = projectSample(sample, vs);
      if (ground_dist < 0.0) continue;
      if (vs == MapManager::VoxelStatus::kUnknown) {
        hanging = true;
      }

      sample[2] -= (ground_dist - planning_params_.max_ground_height);
      state[0] = sample[0];
      state[1] = sample[1];
      state[2] -= (ground_dist - planning_params_.max_ground_height);
    }

    // Check if surrounding area is free.
    if (MapManager::VoxelStatus::kFree ==
        map_manager_->getBoxStatus(
            Eigen::Vector3d(state[0], state[1], state[2]) +
                robot_params_.center_offset,
            robot_box_size_, true)) {
      if (manual_mode) {
        random_sampler_manual_mode_.pushSample(state, true);  // for debug.

      } else {
        random_sampler_.pushSample(state, true);  // for debug.
      }
      found = true;
    } else {
      stat_->num_vertices_fail++;
      if (manual_mode) {
        random_sampler_manual_mode_.pushSample(state, false);  // for debug.

      } else {
        random_sampler_.pushSample(state, false);  // for debug.
      }
    }
  }
  vertex.state = state;
  vertex.is_hanging = hanging;
  return found;
}

void GraphManager::clusterNodes(std::shared_ptr<GraphUtils> g) {
  if (g->getNumVertices() == 0) {
    return;
  }
  // cluster frontier nodes based on relative volumetric gain for efficient
  // insertion into sparse graph
  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->graph_->getVertexIterator(vi);

  int cluster_count = 0;

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->graph_->getVertexProperty(it);
    StateVec v_s = g->vertices_map_[id]->state;
    Vertex* v = g->vertices_map_[id];
    std::vector<Vertex*> nearby_vertices;
    double my_gain = -std::numeric_limits<double>::max();
    g->getNearestVertices(&g->vertices_map_[id]->state,
                          2 * planning_params_.nearest_range, &nearby_vertices);

    Vertex* centroid_vertex;
    for (int i = 0;
         i < nearby_vertices.size() and nearby_vertices[i]->is_leaf_vertex;
         i++) {
      Vertex* nearby_vertex = nearby_vertices[i];
      if (nearby_vertex->cluster_id == -1 and
          nearby_vertex->vol_gain.gain > my_gain) {
        my_gain = nearby_vertex->vol_gain.gain;
        nearby_vertex->cluster_id = cluster_count;
        centroid_vertex = nearby_vertex;
      } else {
        nearby_vertex->cluster_id = cluster_count;
      }
    }
    centroid_vertex->is_cluster_centroid = true;
    cluster_count = cluster_count + 1;
  }
}

std::vector<int> GraphManager::performShortestPathsClustering(
    const std::shared_ptr<GraphUtils> graph_manager,
    const ShortestPathsReport& graph_rep, std::vector<Vertex*>& vertices,
    double dist_threshold, double principle_path_min_length,
    bool refinement_enable) {
  // Asumme long paths are principle paths.
  // Go over one by one from the longest ones.
  // Group into clusters based on the normalized DTW distance metric.
  // Refine by choosing closest & valid principle path.

  // Sort into descending order.
  std::sort(vertices.begin(), vertices.end(),
            [&graph_manager, &graph_rep](const Vertex* a, const Vertex* b) {
              return graph_manager->getShortestDistance(a->id, graph_rep) >
                     graph_manager->getShortestDistance(b->id, graph_rep);
            });

  std::vector<std::vector<Eigen::Vector3d>> cluster_paths;
  std::vector<int> cluster_ids;
  for (int i = 0; i < vertices.size(); ++i) {
    std::vector<Eigen::Vector3d> path_cur;
    graph_manager->getShortestPath(vertices[i]->id, graph_rep, true, path_cur);
    bool found_a_neigbor = false;
    for (int j = 0; j < cluster_paths.size(); ++j) {
      if (Trajectory::compareTwoTrajectories(path_cur, cluster_paths[j],
                                             dist_threshold)) {
        vertices[i]->cluster_id = cluster_ids[j];
        found_a_neigbor = true;
        break;
      }
    }
    if (!found_a_neigbor) {
      // Can not find any neigbor, set this as a new principle path.
      cluster_paths.emplace_back(path_cur);
      cluster_ids.push_back(vertices[i]->id);
      vertices[i]->cluster_id = vertices[i]->id;
    }
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                "Cluster %d paths into %d clusters.", (int)vertices.size(),
                (int)cluster_paths.size());

  // Refinement step, remove short path and choose closest cluster.
  if (refinement_enable) {
    // Clean out noisy paths by removing short path.
    std::vector<std::vector<Eigen::Vector3d>> cluster_paths_refine;
    std::vector<int> cluster_ids_refine;
    for (int j = 0; j < cluster_paths.size(); ++j) {
      double path_len = Trajectory::getPathLength(cluster_paths[j]);
      if (path_len >= principle_path_min_length) {
        cluster_paths_refine.push_back(cluster_paths[j]);
        cluster_ids_refine.push_back(cluster_ids[j]);
      }
    }
    // Recheck and choose closest one.
    for (int i = 0; i < vertices.size(); ++i) {
      std::vector<Eigen::Vector3d> path_cur;
      graph_manager->getShortestPath(vertices[i]->id, graph_rep, true,
                                     path_cur);
      double dist_min = std::numeric_limits<double>::infinity();
      for (int j = 0; j < cluster_paths_refine.size(); ++j) {
        double dist_score = Trajectory::computeDistanceBetweenTwoTrajectories(
            path_cur, cluster_paths_refine[j]);
        if (dist_min > dist_score) {
          dist_min = dist_score;
          vertices[i]->cluster_id = cluster_ids_refine[j];
        }
      }
    }
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO,
                  "Clustering with refinement %d paths into %d clusters.",
                  (int)vertices.size(), (int)cluster_paths_refine.size());
    return cluster_ids_refine;
  } else {
    return cluster_ids;
  }
}

void GraphManager::segmentPath(nav_msgs::Path path, nav_msgs::Path& path_seg) {
  float dt_ = 0.5;
  path_seg.poses.clear();
  double v_max_ = 0.5;
  double yaw_rate_max_ = 0.5;
  if (path.poses.size() == 0) return;

  for (int i = 0; i < (path.poses.size() - 1); ++i) {
    Eigen::Vector3d start(path.poses[i].pose.position.x,
                          path.poses[i].pose.position.y,
                          path.poses[i].pose.position.z);
    Eigen::Vector3d end(path.poses[i + 1].pose.position.x,
                        path.poses[i + 1].pose.position.y,
                        path.poses[i + 1].pose.position.z);
    Eigen::Vector3d distance = end - start;
    double yaw_start = 0;
    double yaw_end = 0;
    double yaw_direction = 0;
    if (yaw_direction > M_PI) {
      yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI) {
      yaw_direction += 2.0 * M_PI;
    }

    double dist_norm = distance.norm();
    double disc = std::min(dt_ * v_max_ / dist_norm,
                           dt_ * yaw_rate_max_ / abs(yaw_direction));

    bool int_flag = true;

    if (int_flag) {
      for (double it = 0.0; it <= 1.0; it += disc) {
        tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                           (1.0 - it) * start[1] + it * end[1],
                           (1.0 - it) * start[2] + it * end[2]);
        double yaw = yaw_start + yaw_direction * it;
        if (yaw > M_PI) yaw -= 2.0 * M_PI;
        if (yaw < -M_PI) yaw += 2.0 * M_PI;
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
}  // segmentPath

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
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                  "[IMPRV] Path too close to obstacle");
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

bool GraphManager::improveFreePath(
    const std::vector<geometry_msgs::Pose>& path_orig,
    std::vector<geometry_msgs::Pose>& path_mod, bool relaxed,
    bool edge_refinement) {
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

  if (edge_refinement) {
    path_mod1.clear();
    path_mod1 = path_orig;
  }

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
        // ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Segment length
        // %f.", segment_len);
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
            ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                          "Combine nodes to remove short segments.");
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
    } else if (robot_params_.type == RobotType::kGroundRobot) {
      std::vector<Eigen::Vector3d> pr1, pr2;
      ProjectedEdgeStatus e_pr1, e_pr2;
      e_pr1 = getProjectedEdgeStatus(p0_mod, p1_parallel, robot_box_size_,
                                     false, pr1, false);
      if (ProjectedEdgeStatus::kAdmissible == e_pr1 ||
          ProjectedEdgeStatus::kSteep == e_pr1)
        e1_admissible = true;
      e_pr2 = getProjectedEdgeStatus(p0_mod, p1, robot_box_size_, false, pr2,
                                     false);
      if (ProjectedEdgeStatus::kAdmissible == e_pr2 ||
          ProjectedEdgeStatus::kSteep == e_pr2)
        e2_admissible = true;
    }
    if (e1_admissible) {
      p1_target = p1_parallel;
    } else if (e2_admissible) {
      p1_target = p1;
    } else {
      ROS_WARN("Segment not free");
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
      } else if (robot_params_.type == RobotType::kGroundRobot) {
        std::vector<Eigen::Vector3d> pr1, pr2;
        ProjectedEdgeStatus es1, es2;
        es1 = getProjectedEdgeStatus(p0_mod, p1_mod, robot_box_size_, false,
                                     pr1, false);
        if (!(ProjectedEdgeStatus::kAdmissible != es1 &&
              (!relaxed || ProjectedEdgeStatus::kSteep != es1)))
          e1_admissible = true;
        es2 = getProjectedEdgeStatus(p1_mod, p2, robot_box_size_, false, pr2,
                                     false);
        if (!(ProjectedEdgeStatus::kAdmissible != es2 &&
              (!relaxed || ProjectedEdgeStatus::kSteep != es2)))
          e2_admissible = true;
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
  if ((mod_success) && (planning_params_.yaw_tangent_correction)) {
    for (int i = 1; i < path_mod.size(); ++i) {
      Eigen::Vector3d vec(path_mod[i].position.x - path_mod[i - 1].position.x,
                          path_mod[i].position.y - path_mod[i - 1].position.y,
                          path_mod[i].position.z - path_mod[i - 1].position.z);
      if (planning_params_.planning_backward) vec = -vec;
      double yawhalf = 0.5 * std::atan2(vec[1], vec[0]);
      path_mod[i].orientation.x = 0.0;
      path_mod[i].orientation.y = 0.0;
      path_mod[i].orientation.z = sin(yawhalf);
      path_mod[i].orientation.w = cos(yawhalf);
    }
  }

  return mod_success;
}

double GraphManager::projectSample(Eigen::Vector3d& sample,
                                   MapManager::VoxelStatus& voxel_status) {
  double max_proj_len = 5.0;

  float voxel_size = map_manager_->getResolution();

  int unknown_count = 0;
  double central_ray_len = 0.0;
  std::vector<Eigen::Vector3d> extra_samples(5, Eigen::Vector3d::Zero());
  extra_samples[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
  extra_samples[1] = Eigen::Vector3d(0.5, 0.0, 0.5);
  extra_samples[2] = Eigen::Vector3d(-0.5, 0.0, 0.5);
  extra_samples[3] = Eigen::Vector3d(0.0, 0.5, 0.5);
  extra_samples[4] = Eigen::Vector3d(0.0, -0.5, 0.5);
  for (int i = 0; i < extra_samples.size(); ++i) {
    Eigen::Vector3d delta = extra_samples[i];
    Eigen::Vector3d start = sample + delta;
    Eigen::Vector3d end = start - Eigen::Vector3d(0.0, 0.0, max_proj_len);

    if (planning_params_.interpolate_projection_distance) {
      double ground_dist = 0.0;
      Eigen::Vector3d point = start;
      while (ground_dist < max_proj_len) {
        double tsdf_dist = map_manager_->getPointDistance(point);
        if (tsdf_dist < 0.0) {
          // Hit Unknown
          if (i == 0) {
            central_ray_len = (point - start).norm();
          }
          ++unknown_count;
          break;
        } else {
          if (tsdf_dist <= voxel_size) {
            // Hit occupied
            voxel_status = MapManager::VoxelStatus::kOccupied;
            sample(0) = start(0);
            sample(1) = start(1);
            return ground_dist + tsdf_dist;
          } else {
            // Still in free
            ground_dist += tsdf_dist;
            point -= Eigen::Vector3d(0.0, 0.0, tsdf_dist - 0.02);
          }
        }
      }
    } else {
      Eigen::Vector3d end_voxel;
      double tsdf_dist;
      MapManager::VoxelStatus vs =
          map_manager_->getRayStatus(start, end, true, end_voxel, tsdf_dist);

      if (vs == MapManager::VoxelStatus::kOccupied) {
        double ray_len = std::abs(start(2) - end_voxel(2));
        if (i == 0) {
          central_ray_len = ray_len;
        }
        voxel_status = MapManager::VoxelStatus::kOccupied;
        sample(0) = start(0);
        sample(1) = start(1);
        return ray_len;
      } else if (vs == MapManager::VoxelStatus::kUnknown) {
        double ray_len = std::abs(start(2) - end_voxel(2));
        if (i == 0) {
          central_ray_len = ray_len;
        }
        ++unknown_count;
      }
    }
  }

  if (unknown_count >= extra_samples.size()) {
    voxel_status = MapManager::VoxelStatus::kUnknown;
    return central_ray_len;
  }

  voxel_status = MapManager::VoxelStatus::kFree;
  return -1.0;
}

ProjectedEdgeStatus GraphManager::getProjectedEdgeStatus(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
    std::vector<Eigen::Vector3d>& projected_edge_out, bool is_hanging) {
  double step_size = 2.0 * map_manager_->getResolution();
  double max_inclination = planning_params_.max_inclination;

  std::vector<Eigen::Vector3d> projected_edge;
  Eigen::Vector3d ray = end - start;
  double edge_incl = std::atan2(std::abs(ray(2)), std::abs(ray.head(2).norm()));
  if (edge_incl > max_inclination) {
    return ProjectedEdgeStatus::kSteep;
  }

  double ray_len = ray.norm();
  Eigen::Vector3d ray_normed = ray / ray_len;

  if ((end - start).norm() >= 2 * step_size) {
    Eigen::Vector3d last_point;
    for (double step = 0.0; step < ray_len; step += step_size) {
      Eigen::Vector3d edge_point = start + step * ray_normed;
      last_point = edge_point;
      MapManager::VoxelStatus vs;
      double ground_height = projectSample(edge_point, vs);
      // Only allow intermediate hanging vertices if either of the end vertices
      // are hanging:
      if (vs == MapManager::VoxelStatus::kUnknown || ground_height < 0.0) {
        if (!is_hanging) {
          return ProjectedEdgeStatus::kHanging;
        }
      }
      Eigen::Vector3d projected_edge_pt = edge_point;
      // Get the point on the ground at a heigh of
      // planning_params_.max_ground_height
      projected_edge_pt(2) -=
          (ground_height - planning_params_.max_ground_height);
      projected_edge.push_back(projected_edge_pt);
    }
    if ((last_point - end).norm() < 0.75 * step_size) {
      projected_edge.erase(projected_edge.end());
    }
  } else {
    MapManager::VoxelStatus vs;
    Eigen::Vector3d start_m = start;
    Eigen::Vector3d end_m = end;
    double ground_height = projectSample(start_m, vs);
    // Project start and end points (extra check)
    if (vs == MapManager::VoxelStatus::kUnknown || ground_height < 0.0) {
      if (!is_hanging) {
        return ProjectedEdgeStatus::kHanging;
      }
    }
    ground_height = projectSample(end_m, vs);
    if (vs == MapManager::VoxelStatus::kUnknown || ground_height < 0.0) {
      if (!is_hanging) {
        return ProjectedEdgeStatus::kHanging;
      }
    }
    projected_edge.push_back(start);
  }

  MapManager::VoxelStatus vs;
  Eigen::Vector3d end_m = end;
  double ground_height = projectSample(end_m, vs);
  if (vs == MapManager::VoxelStatus::kUnknown || ground_height < 0.0) {
    if (!is_hanging) {
      return ProjectedEdgeStatus::kHanging;
    }
  }
  projected_edge.push_back(end_m);

  // Check edge segment inclination. Cheaper than collision check
  for (int i = 1; i < projected_edge.size(); ++i) {
    Eigen::Vector3d segment = projected_edge[i] - projected_edge[i - 1];
    double theta =
        std::atan2(std::abs(segment(2)), std::abs(segment.head(2).norm()));
    if (std::abs(theta) > max_inclination) {
      return ProjectedEdgeStatus::kSteep;
    }
  }

  // Check edge collision:
  for (int i = 1; i < projected_edge.size(); ++i) {
    MapManager::VoxelStatus vs =
        map_manager_->getPathStatus(projected_edge[i - 1], projected_edge[i],
                                    box_size, stop_at_unknown_voxel);
    if (vs == MapManager::VoxelStatus::kUnknown) {
      return ProjectedEdgeStatus::kUnknown;
    } else if (vs == MapManager::VoxelStatus::kOccupied) {
      return ProjectedEdgeStatus::kOccipied;
    }
  }

  // Edge is admissible
  projected_edge_out = projected_edge;
  return ProjectedEdgeStatus::kAdmissible;
}

void GraphManager::improvePath(std::vector<Vertex*> raw_path,
                               std::vector<Vertex*>& path) {
  // Reset the improvement graph & clear memory

  improvement_graph_.reset(new GraphUtils());
  if (improvement_graph_ != NULL) {
    improvement_graph_->reset();
  }
  // Reset SP report
  improvement_graph_rep_.reset();

  // Include root vertex as current robot state
  Vertex* root_vertex =
      new Vertex(improvement_graph_->generateVertexID(), robotState_);
  improvement_graph_->addVertex(root_vertex);
  root_vertex->is_root = true;

  // Include goal vertex as last vertex of Dijkstra path

  Vertex* goal_vertex = new Vertex(improvement_graph_->generateVertexID(),
                                   raw_path.back()->state);
  improvement_graph_->addVertex(goal_vertex);

  // Not ideal, but check nearest neighbours to add edges in a graph that will
  // improve the path by shortcutting some edges

  for (std::vector<Vertex*>::iterator it_vert = raw_path.begin();
       it_vert != std::prev(raw_path.end(), 0); it_vert++) {
    Vertex* imp_vertex =
        new Vertex(improvement_graph_->generateVertexID(), (*it_vert)->state);
    improvement_graph_->addVertex(imp_vertex);
  }

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  improvement_graph_->graph_->getVertexIterator(vi);

  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = improvement_graph_->graph_->getVertexProperty(it);

    // Get nearby vertices from the current vertex and connect the them to form
    // local edges
    std::vector<Vertex*> nearby_vertices;
    improvement_graph_->getNearestVertices(
        &improvement_graph_->vertices_map_[id]->state,
        2 * planning_params_.nearest_range, &nearby_vertices);

    for (int i = 0; i < nearby_vertices.size(); i++) {
      Eigen::Vector3d v1(improvement_graph_->vertices_map_[id]->state[0],
                         improvement_graph_->vertices_map_[id]->state[1],
                         improvement_graph_->vertices_map_[id]->state[2]);

      Eigen::Vector3d v2(nearby_vertices[i]->state[0],
                         nearby_vertices[i]->state[1],
                         nearby_vertices[i]->state[2]);
      MapManager::VoxelStatus vn =
          map_manager_->getPathStatus(v1, v2, robot_box_size_, true);

      double edge_length = improvement_graph_->distanceBetweenVertices(
          improvement_graph_->vertices_map_[id], nearby_vertices[i]);

      if (edge_length < 2 * planning_params_.edge_length_max and
          MapManager::VoxelStatus::kFree == vn) {
        improvement_graph_->addEdge(
            improvement_graph_->vertices_map_[id], nearby_vertices[i],
            improvement_graph_->distanceBetweenVertices(
                improvement_graph_->vertices_map_[id], nearby_vertices[i]));
      }
    }
  }

  // Dijkstra SP report from root to all vertices in improvement graph

  if (improvement_graph_->findShortestPaths(root_vertex->id,
                                            improvement_graph_rep_)) {
  } else {
    ROS_WARN("Could not find dijkstra shortest paths on graph improvement \n");
  }

  improvement_graph_->getShortestPath(goal_vertex->id, improvement_graph_rep_,
                                      true, path);
}

void GraphManager::expandGraph(std::shared_ptr<GraphUtils> graph_manager,
                               Vertex& new_vertex, ExpandGraphReport& rep,
                               bool allow_short_edge) {
  // Find nearest neighbour
  StateVec new_state;
  new_state = new_vertex.state;

  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&new_state, &nearest_vertex)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  if (nearest_vertex == NULL) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }

  // Check for collision of new connection plus some overshoot distance.
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                            new_state[2] - origin[2]);
  double direction_norm = direction.norm();

  if (direction_norm > planning_params_.edge_length_max) {
    direction = planning_params_.edge_length_max * direction.normalized();
  } else if ((!allow_short_edge) &&
             (direction_norm <= planning_params_.edge_length_min)) {
    // Should not add short edge.
    rep.status = ExpandGraphStatus::kErrorShortEdge;
    return;
  }

  // Recalculate the distance.
  direction_norm = direction.norm();
  new_state[0] = origin[0] + direction[0];
  new_state[1] = origin[1] + direction[1];
  new_state[2] = origin[2] + direction[2];

  if (robot_params_.type == RobotType::kGroundRobot) {
    Eigen::Vector3d new_pos;
    new_pos << new_state[0], new_state[1], new_state[2];
    MapManager::VoxelStatus vs;
    double ground_height = projectSample(new_pos, vs);
    if (vs == MapManager::VoxelStatus::kOccupied) {
      new_pos[2] -= (ground_height - planning_params_.max_ground_height);
    } else {
      rep.status = ExpandGraphStatus::kErrorCollisionEdge;
      return;
    }
    new_state[0] = new_pos[0];
    new_state[1] = new_pos[1];
    new_state[2] = new_pos[2];
  }

  // Since we are buiding graph,
  // Consider to check the overshoot for both 2 directions except root node.
  Eigen::Vector3d overshoot_vec =
      planning_params_.edge_overshoot * direction.normalized();
  Eigen::Vector3d start_pos = origin + robot_params_.center_offset;
  if (nearest_vertex->id != 0) start_pos = start_pos - overshoot_vec;
  Eigen::Vector3d end_pos =
      origin + robot_params_.center_offset + direction + overshoot_vec;
  if (planning_params_.geofence_checking_enable &&
      (GeofenceManager::CoordinateStatus::kViolated ==
       geofence_manager_->getPathStatus(
           Eigen::Vector2d(start_pos[0], start_pos[1]),
           Eigen::Vector2d(end_pos[0], end_pos[1]),
           Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
    rep.status = ExpandGraphStatus::kErrorGeofenceViolated;
    return;
  }

  bool admissible_edge = false;
  int steep_edges = 0;
  std::vector<Eigen::Vector3d> projected_edge;
  if (robot_params_.type == RobotType::kAerialRobot) {
    MapManager::VoxelStatus vs =
        map_manager_->getPathStatus(start_pos, end_pos, robot_box_size_, true);
    if (MapManager::VoxelStatus::kFree == vs) {
      admissible_edge = true;
    }
  } else if (robot_params_.type == RobotType::kGroundRobot) {
    bool is_hanging = nearest_vertex->is_hanging || new_vertex.is_hanging;
    ProjectedEdgeStatus es = getProjectedEdgeStatus(
        start_pos, end_pos, robot_box_size_, true, projected_edge, is_hanging);
    if (ProjectedEdgeStatus::kAdmissible == es) {
      admissible_edge = true;

      StateVec strt_st(projected_edge[0](0), projected_edge[0](1),
                       projected_edge[0](2), 0.0);
      Vertex* strt_vert =
          new Vertex(projected_graph_->generateVertexID(), strt_st);
      projected_graph_->addVertex(strt_vert);
      Vertex* prev_vert = strt_vert;
      for (int i = 1; i < projected_edge.size(); ++i) {
        StateVec proj_st(projected_edge[i](0), projected_edge[i](1),
                         projected_edge[i](2), 0.0);
        Vertex* proj_vert =
            new Vertex(projected_graph_->generateVertexID(), proj_st);
        projected_graph_->addVertex(proj_vert);
        double edge_len = (proj_vert->state - prev_vert->state).norm();
        projected_graph_->addEdge(proj_vert, prev_vert, edge_len);
        prev_vert = proj_vert;
      }
    } else if (ProjectedEdgeStatus::kSteep == es)
      ++steep_edges;
  }

  if (admissible_edge) {
    Vertex* new_vertex_ptr =
        new Vertex(graph_manager->generateVertexID(), new_state);
    // new_vertex_ptr->id = graph_manager->generateVertexID();
    new_vertex_ptr->state = new_state;
    // Form a tree as the first step.
    new_vertex_ptr->parent = nearest_vertex;
    new_vertex_ptr->distance = nearest_vertex->distance + direction_norm;
    new_vertex_ptr->is_hanging = new_vertex.is_hanging;
    nearest_vertex->children.push_back(new_vertex_ptr);
    graph_manager->addVertex(new_vertex_ptr);
    ++rep.num_vertices_added;
    rep.vertex_added = new_vertex_ptr;
    graph_manager->addEdge(new_vertex_ptr, nearest_vertex, direction_norm);
    ++rep.num_edges_added;
    if (local_exploration_ongoing_) {
      double max_inclination = 0.0;
      double avg_inclination = 0.0;

      for (int i = 1; i < projected_edge.size(); ++i) {
        Eigen::Vector3d edge_segment =
            projected_edge[i] - projected_edge[i - 1];
        double inclination = (std::atan2(std::abs(edge_segment(2)),
                                         edge_segment.head(2).norm()));
        // std::cout << inclination << std::endl;
        if (inclination > max_inclination) max_inclination = inclination;
        avg_inclination += inclination;
      }
      avg_inclination /= projected_edge.size();

      edge_inclinations_[new_vertex_ptr->id][nearest_vertex->id] =
          avg_inclination;
      edge_inclinations_[nearest_vertex->id][new_vertex_ptr->id] =
          avg_inclination;
    }

    // Form more edges from neighbors if set RRG mode.
    if (planning_params_.rr_mode == RRModeType::kGraph) {
      std::vector<Vertex*> nearest_vertices;
      if (!graph_manager->getNearestVertices(
              &new_state, planning_params_.nearest_range, &nearest_vertices)) {
        rep.status = ExpandGraphStatus::kErrorKdTree;
        return;
      }
      origin << new_vertex_ptr->state[0], new_vertex_ptr->state[1],
          new_vertex_ptr->state[2];
      for (int i = 0; i < nearest_vertices.size(); ++i) {
        direction << nearest_vertices[i]->state[0] - origin[0],
            nearest_vertices[i]->state[1] - origin[1],
            nearest_vertices[i]->state[2] - origin[2];
        double d_norm = direction.norm();

        if ((d_norm > planning_params_.nearest_range_min) &&
            (d_norm < planning_params_.nearest_range_max)) {
          Eigen::Vector3d p_overshoot =
              direction / d_norm * planning_params_.edge_overshoot;
          Eigen::Vector3d p_start =
              origin + robot_params_.center_offset - p_overshoot;
          Eigen::Vector3d p_end =
              origin + robot_params_.center_offset + direction;
          if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;

          bool geofence_pass = true;
          if (planning_params_.geofence_checking_enable &&
              (GeofenceManager::CoordinateStatus::kViolated ==
               geofence_manager_->getPathStatus(
                   Eigen::Vector2d(p_start[0], p_start[1]),
                   Eigen::Vector2d(p_end[0], p_end[1]),
                   Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
            geofence_pass = false;
          }

          if (geofence_pass) {
            admissible_edge = false;
            if (robot_params_.type == RobotType::kAerialRobot) {
              MapManager::VoxelStatus vs = map_manager_->getPathStatus(
                  p_start, p_end, robot_box_size_, true);
              if (MapManager::VoxelStatus::kFree == vs) {
                admissible_edge = true;
              }
            } else if (robot_params_.type == RobotType::kGroundRobot) {
              projected_edge.clear();
              bool is_hanging =
                  new_vertex_ptr->is_hanging || nearest_vertices[i]->is_hanging;
              ProjectedEdgeStatus es =
                  getProjectedEdgeStatus(p_start, p_end, robot_box_size_, true,
                                         projected_edge, is_hanging);
              if (ProjectedEdgeStatus::kAdmissible == es) {
                admissible_edge = true;
                StateVec strt_st(projected_edge[0](0), projected_edge[0](1),
                                 projected_edge[0](2), 0.0);
                Vertex* strt_vert =
                    new Vertex(projected_graph_->generateVertexID(), strt_st);
                projected_graph_->addVertex(strt_vert);
                Vertex* prev_vert = strt_vert;
                for (int i = 1; i < projected_edge.size(); ++i) {
                  StateVec proj_st(projected_edge[i](0), projected_edge[i](1),
                                   projected_edge[i](2), 0.0);
                  Vertex* proj_vert =
                      new Vertex(projected_graph_->generateVertexID(), proj_st);
                  projected_graph_->addVertex(proj_vert);
                  double edge_len =
                      (proj_vert->state - prev_vert->state).norm();
                  projected_graph_->addEdge(proj_vert, prev_vert, edge_len);
                  prev_vert = proj_vert;
                }
              } else if (ProjectedEdgeStatus::kSteep == es)
                ++steep_edges;
            }

            if (admissible_edge) {
              if (local_exploration_ongoing_) {
                double max_inclination = 0.0;
                double avg_inclination = 0.0;
                for (int i = 1; i < projected_edge.size(); ++i) {
                  Eigen::Vector3d edge_segment =
                      projected_edge[i] - projected_edge[i - 1];
                  double inclination = (std::atan2(
                      std::abs(edge_segment(2)), edge_segment.head(2).norm()));
                  if (inclination > max_inclination)
                    max_inclination = inclination;
                  avg_inclination += inclination;
                }
                avg_inclination /= projected_edge.size();
                edge_inclinations_[new_vertex_ptr->id]
                                  [nearest_vertices[i]->id] = avg_inclination;
                edge_inclinations_[nearest_vertices[i]->id]
                                  [new_vertex_ptr->id] = avg_inclination;
              }
              graph_manager->addEdge(new_vertex_ptr, nearest_vertices[i],
                                     d_norm);
              ++rep.num_edges_added;
            }
          }
        }
      }
    }
  } else {
    stat_->num_edges_fail++;
    if (stat_->num_edges_fail < 500) {
      std::vector<double> vtmp = {start_pos[0], start_pos[1], start_pos[2],
                                  end_pos[0],   end_pos[1],   end_pos[2]};
      stat_->edges_fail.push_back(vtmp);
    }
    rep.status = ExpandGraphStatus::kErrorCollisionEdge;
    return;
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

GraphManager::GraphStatus GraphManager::buildGraph(
    std::shared_ptr<GraphUtils> graph, bool manual_mode) {
  int loop_count = 0;
  int num_vertices = 1;
  int num_edges = 0;
  int loop_count_max = 0;
  int num_vertices_max = 1;
  int num_edges_max = 0;
  GraphManager::GraphStatus build_graph_status =
      GraphManager::GraphStatus::NOT_OK;

  // For manual mode scale max vertices and edges to match  larger sampling
  // space
  if (manual_mode) {
    loop_count_max = 2 * planning_params_.num_loops_max;
    num_vertices_max = 4 * planning_params_.num_vertices_max;
    num_edges_max = 2 * planning_params_.num_edges_max;
  } else {
    loop_count_max = planning_params_.num_loops_max;
    num_vertices_max = planning_params_.num_vertices_max;
    num_edges_max = planning_params_.num_edges_max;
  }

  resetProjectedGraph();

  local_exploration_ongoing_ = true;

  while ((loop_count++ < loop_count_max) && (num_vertices < num_vertices_max) &&
         (num_edges < num_edges_max)) {
    Vertex new_vertex(-1, StateVec::Zero());

    if (!sampleVertex(new_vertex, manual_mode)) {
      continue;
    }
    ExpandGraphReport rep;
    expandGraph(graph, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
      local_exploration_ongoing_ = false;
    }

    if ((loop_count >= planning_params_.num_loops_cutoff) &&
        (graph->getNumVertices() <= 1)) {
      break;
    }
  }

  if (graph->getNumVertices() > 2 and graph->getNumEdges() > 1) {
    build_graph_status = GraphManager::GraphStatus::OK;
  }

  // stat_->build_graph_time = GET_ELAPSED_TIME(ttime);
  // t2 = std::chrono::high_resolution_clock::now();

  return build_graph_status;
}

nav_msgs::Path GraphManager::runPlannerkManualMode(StateVec s) {
  // ROS_WARN_STREAM("robotState_ : " << robotState_[0] << " ," <<
  // robotState_[1]
  //                                  << " ," << robotState_[2] << " ,"
  //                                  << robotState_[3] << "\n");

  GraphManager::GraphStatus local_planner_status =
      GraphManager::GraphStatus::NOT_OK;
  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  GraphManager::resetLocalGraph();
  local_dijkstra_paths_.clear();

  std::vector<Vertex*> path;
  path.clear();
  nav_msgs::Path path_to_goal;
  path_to_goal.poses.clear();
  geometry_msgs::PoseStamped p;
  Vertex* vertex = nullptr;
  local_root_vertex = new Vertex(local_graph_->generateVertexID(), robotState_);
  local_root_vertex->is_root = true;
  local_graph_->addVertex(local_root_vertex);
  bool manual_mode = true;
  GraphManager::GraphStatus build_graph_status =
      GraphManager::buildGraph(local_graph_, manual_mode);
  switch (build_graph_status) {
    case GraphManager::GraphStatus::OK:
      ROS_DEBUG("[kManual Mode LOCAL GRAPH STATUS] SG build : OK ");

      // Fetch shortest path report from root vertex to all vertices in the
      // graph
      if (!local_graph_->findShortestPaths(local_root_vertex->id,
                                           local_graph_rep_)) {
        ROS_WARN(
            "kManual Mode : Could not find dijkstra shortest paths on the "
            "graph \n");
      }

      local_graph_->getNearestVertex(&s, &vertex);

      ROS_WARN_STREAM("Nearest vertex : "
                      << vertex->state[0] << " ," << vertex->state[1] << " ,"
                      << vertex->state[2] << " ," << vertex->state[3] << "\n");
      local_graph_->getShortestPath(vertex->id, local_graph_rep_, true, path);
      if (!path.empty()) {
        best_path_.clear();
        GraphManager::improvePath(path, best_path_);
        if (best_path_.empty()) {
          best_path_ = path;
        }
        local_planner_status = GraphManager::GraphManager::OK;
      } else {
        ROS_WARN("manual mode could not plan path \n");
      }
      break;
    case GraphManager::GraphStatus::ERR_KDTREE:
      ROS_WARN(
          "[kManual Mode PLANNER_ERROR] An issue occurred with kdtree data.");
      break;
    case GraphManager::GraphStatus::NOT_OK:
      ROS_WARN("[kManual Mode LOCAL GRAPH STATUS] : NOT_OK");
      break;
    default:
      ROS_WARN(
          "[kManual Mode PLANNER_ERROR] Error occurred in global planning.");
      break;
  }

  if (local_planner_status == GraphManager::GraphManager::OK) {
    p.header.stamp = ros::Time::now();
    p.header.frame_id = world_frame_id;
    path_to_goal.header.stamp = ros::Time::now();
    path_to_goal.header.frame_id = world_frame_id;
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

    bool improvement_success = GraphManager::improveFreePath(
        original_path, modified_path, true, false);
    if (!improvement_success) {
      modified_path.clear();
      modified_path = original_path;
    }
    geometry_msgs::PoseStamped pose_mod;
    pose_mod.header.frame_id = world_frame_id;
    pose_mod.header.stamp = ros::Time::now();

    for (int i = 0; i < modified_path.size(); ++i) {
      float angle;
      if (i == 0) {
        angle = robotState_[3];
      } else {
        angle = atan2(
            modified_path[i].position.y - modified_path[i - 1].position.y,
            modified_path[i].position.x - modified_path[i - 1].position.x);
      }
      Eigen::Vector3d v(0, 0, angle);
      Eigen::Quaternion<float> q = Euler2Quaternion(v);
      pose_mod.pose.position.x = modified_path[i].position.x;
      pose_mod.pose.position.y = modified_path[i].position.y;
      pose_mod.pose.position.z = modified_path[i].position.z;
      pose_mod.pose.orientation.x = q.x();
      pose_mod.pose.orientation.y = q.y();
      pose_mod.pose.orientation.z = q.z();
      pose_mod.pose.orientation.w = q.w();
      path_to_goal.poses.push_back(pose_mod);
    }

    visualization_server_->visualizeLocalGraph(local_graph_);
    visualization_server_->visualizeExplorationPath(path_to_goal);
  }
  // else {
  //   runPlannerkManualMode(s);
  // }
  ROS_INFO_STREAM("MG graph has " << local_graph_->getNumVertices()
                                  << " nodes & " << local_graph_->getNumEdges()
                                  << " edges\n");

  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total =
      duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  ROS_INFO_STREAM("MG --> build time " << duration_total.count() << " ms \n");

  return path_to_goal;
}  // runPlannerkManualMode

GraphManager::GraphStatus GraphManager::runLocalPlanner() {
  GraphManager::GraphStatus local_planner_status =
      GraphManager::GraphStatus::NOT_OK;
  int frontier_vertices_count = 0;
  high_resolution_clock::time_point start_total = high_resolution_clock::now();

  // GraphManager::updateGlobalGraph();
  GraphManager::resetLocalGraph();
  local_dijkstra_paths_.clear();

  local_root_vertex = new Vertex(local_graph_->generateVertexID(), robotState_);
  local_root_vertex->is_root = true;
  local_graph_->addVertex(local_root_vertex);

  bool manual_mode = false;
  GraphManager::GraphStatus build_graph_status =
      GraphManager::buildGraph(local_graph_, manual_mode);
  switch (build_graph_status) {
    case GraphManager::GraphStatus::OK:
      ROS_DEBUG("[LOCAL GRAPH STATUS] SG build : OK ");
      if (GraphManager::extractShortestPaths(local_graph_, local_graph_rep_,
                                             local_root_vertex)) {
        local_planner_status =
            GraphManager::planLocalPath(local_dijkstra_paths_);
      } else {
        local_planner_status = GraphManager::GraphStatus::NO_GAIN;
      }
      break;
    case GraphManager::GraphStatus::ERR_KDTREE:
      ROS_WARN("[PLANNER_ERROR] An issue occurred with kdtree data.");
      break;
    case GraphManager::GraphStatus::NOT_OK:
      ROS_WARN("[LOCAL GRAPH STATUS] : NOT_OK");
      break;
    default:
      ROS_WARN("[PLANNER_ERROR] Error occurred in global planning.");
      break;
  }

  if (local_planner_status == GraphManager::GraphManager::OK) {
    // Reuse the local dijkstra paths to expand the global graph.

    GraphManager::expandGlobalGraph(local_dijkstra_paths_);
  }

  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total =
      duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  ROS_DEBUG_STREAM("SG --> build time " << duration_total.count() << " ms \n");

  // ROS_DEBUG_STREAM("Sub graph has " << local_graph_->getNumVertices() << "
  // nodes & " << local_graph_->getNumEdges() << " edges\n");

  return local_planner_status;

}  // runLocalPlanner

nav_msgs::Path GraphManager::getLocalExplorationPath() {
  return local_exploration_path_;
}

nav_msgs::Path GraphManager::getGlobalExplorationPath() {
  return global_exploration_path_;
}

bool GraphManager::extractShortestPaths(std::shared_ptr<GraphUtils> graph,
                                        ShortestPathsReport rep, Vertex* root) {
  bool path_status = false;
  // Fetch shortest path report from root vertex to all vertices in the graph
  if (!graph->findShortestPaths(root->id, rep)) {
    ROS_WARN("Could not find dijkstra shortest paths on the graph \n");
    return path_status;
  }

  local_dijkstra_paths_.clear();
  // dijkstra_paths.clear();
  graph->findLeafVertices(rep);

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  graph->graph_->getVertexIterator(vi);

  if (graph->leaf_vertices_ids_.empty()) {
    return false;
  }

  std::vector<Vertex*> frontier_vertices;
  // ROS_DEBUG_STREAM ("Total vertices : " << graph->getNumVertices() << " \t
  // Leaf vertices " << graph->leaf_vertices_ids_.size() << "\n");

  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  for (int id : graph->leaf_vertices_ids_) {
    Vertex* vertex = graph->vertices_map_[id];
    vertex->is_leaf_vertex = true;
    GraphManager::computeVolumetricGainRayCasting(vertex->state,
                                                  vertex->vol_gain);
    if (vertex->vol_gain.is_frontier) {
      graph->frontier_vertices_ids_.push_back(vertex->id);
      frontier_vertices.push_back(vertex);
    }
  }
  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total =
      duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  ROS_DEBUG_STREAM("SG --> extract frontiers & shortest paths time "
                   << duration_total.count() << " ms \n");

  std::vector<int> cluster_ids = performShortestPathsClustering(
      graph, rep, frontier_vertices, planning_params_.clustering_radius,
      planning_params_.clustering_radius, true);
  // std::vector<std::vector<Vertex *>> clustered_paths;
  std::vector<Vertex*> path_to_cid;
  local_dijkstra_paths_.clear();
  path_to_cid.clear();

  for (int cid : cluster_ids) {
    path_to_cid.clear();
    graph->getShortestPath(cid, rep, true, path_to_cid);
    local_dijkstra_paths_.push_back(path_to_cid);
    /*
    for (int k = 0; k < path_to_cid.size(); ++k) {
      ROS_DEBUG_STREAM("path point " << k << " >> " << path_to_cid[k]->state[0]
                                     << ", " << path_to_cid[k]->state[1] << ", "
                                     << path_to_cid[k]->state[2] << "\n");
    }
    */
  }

  visualization_server_->visualizeLocalGraph(local_graph_);

  if (!local_dijkstra_paths_.empty()) {
    path_status = true;
  } else {
    ROS_DEBUG_STREAM("local dijkstra paths do not exist \n");
    return false;
  }

  return path_status;
}

GraphManager::GraphStatus GraphManager::planLocalPath(
    std::vector<std::vector<Vertex*>> dijkstra_paths) {
  local_exploration_path_.poses.clear();
  GraphManager::GraphStatus path_status =
      GraphManager::GraphStatus::ERR_NO_FEASIBLE_PATH;
  local_paths_.clear();
  std::pair<std::vector<Vertex*>, double> path_pair;
  if (!dijkstra_paths.empty()) {
    for (int i = 0; i < dijkstra_paths.size(); ++i) {
      path_pair =
          std::make_pair(dijkstra_paths[i], getPathLength(dijkstra_paths[i]));
      local_paths_.push_back(path_pair);
    }
  } else {
    ROS_WARN_STREAM("SG -> No dijkstra paths :( \n");
    return path_status;
  }

  std::vector<std::pair<std::vector<Vertex*>, double>> paths_unordered;
  paths_unordered.clear();

  if (!local_paths_.empty()) {
    visualization_server_->visualizeLocalPaths(local_paths_);

    double gain, path_length, reward;
    double my_reward = -std::numeric_limits<double>::max();

    for (auto i = 0; i < local_paths_.size(); i++) {
      gain = 100 * local_paths_[i].first.back()->vol_gain.unknown_percentage;

      path_length = local_paths_[i].second;
      double ang = getRelativeAngle(local_paths_[i].first);
      // ROS_DEBUG_STREAM ("Vertex Gain --> " << gain << "\t Angle --> " << ang
      // << "\t Path length --> " << path_length << "\n");
      reward = gain - 10 * exp(ang * M_PI / 180);
      // ROS_DEBUG_STREAM ("new reward = " << reward << "\n");

      std::pair<std::vector<Vertex*>, double> path_reward_pair =
          std::make_pair(local_paths_[i].first, reward);
      paths_unordered.push_back(path_reward_pair);
    }

    // sort the paths in descending order based on the leaf vertices exploration
    // reward.

    std::sort(paths_unordered.begin(), paths_unordered.end(),
              [](const std::pair<std::vector<Vertex*>, double>& a,
                 const std::pair<std::vector<Vertex*>, double>& b) {
                return a.second > b.second;
              });

    // ROS_DEBUG_STREAM ("first and last element reward << " <<
    // paths_unordered.front().second << " , " << paths_unordered.back().second
    // << "\n");

    for (int j = 0; j < paths_unordered.size(); j++) {
      std::vector<Vertex*> path = paths_unordered[j].first;
      geometry_msgs::PoseStamped raw_path_pose;
      raw_path_pose.header.frame_id = world_frame_id;
      raw_path_pose.header.stamp = ros::Time::now();
      nav_msgs::Path raw_path;
      raw_path.poses.clear();
      for (int i = 0; i < path.size(); ++i) {
        raw_path_pose.pose.position.x = path[i]->state[0];
        raw_path_pose.pose.position.y = path[i]->state[1];
        raw_path_pose.pose.position.z = path[i]->state[2];
        raw_path_pose.pose.orientation.x = 0;
        raw_path_pose.pose.orientation.y = 0;
        raw_path_pose.pose.orientation.z = 0.2;
        raw_path_pose.pose.orientation.w = 1;
        raw_path.poses.push_back(raw_path_pose);
      }
      visualization_server_->visualizeRawPath(raw_path);

      best_path_.clear();
      GraphManager::improvePath(path, best_path_);
      if (best_path_.empty()) {
        best_path_ = path;
      }
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

      bool improvement_success = GraphManager::improveFreePath(
          original_path, modified_path, true, false);

      if (improvement_success) {
        geometry_msgs::PoseStamped pose_mod;
        pose_mod.header.frame_id = world_frame_id;
        pose_mod.header.stamp = ros::Time::now();
        local_exploration_path_.header.frame_id = world_frame_id;
        local_exploration_path_.header.stamp = ros::Time::now();

        for (int i = 0; i < modified_path.size(); ++i) {
          float angle;
          if (i == 0) {
            angle = robotState_[3];
          } else {
            angle = atan2(
                modified_path[i].position.y - modified_path[i - 1].position.y,
                modified_path[i].position.x - modified_path[i - 1].position.x);
          }
          Eigen::Vector3d v(0, 0, angle);
          Eigen::Quaternion<float> q = Euler2Quaternion(v);
          pose_mod.pose.position.x = modified_path[i].position.x;
          pose_mod.pose.position.y = modified_path[i].position.y;
          pose_mod.pose.position.z = modified_path[i].position.z;
          pose_mod.pose.orientation.x = q.x();
          pose_mod.pose.orientation.y = q.y();
          pose_mod.pose.orientation.z = q.z();
          pose_mod.pose.orientation.w = q.w();
          local_exploration_path_.poses.push_back(pose_mod);
        }

        visualization_server_->visualizeExplorationPath(
            local_exploration_path_);
        break;
      }

      else if (j == (paths_unordered.size() - 1)) {
        ROS_WARN("SG --> Improved local path is empty \n");
        geometry_msgs::PoseStamped pose_mod;
        pose_mod.header.frame_id = world_frame_id;
        pose_mod.header.stamp = ros::Time::now();
        local_exploration_path_.header.frame_id = world_frame_id;
        local_exploration_path_.header.stamp = ros::Time::now();

        for (int i = 0; i < original_path.size(); ++i) {
          float angle;
          if (i == 0) {
            angle = robotState_[3];
          } else {
            angle = atan2(
                original_path[i].position.y - original_path[i - 1].position.y,
                original_path[i].position.x - original_path[i - 1].position.x);
          }
          Eigen::Vector3d v(0, 0, angle);
          Eigen::Quaternion<float> q = Euler2Quaternion(v);
          pose_mod.pose.position.x = original_path[i].position.x;
          pose_mod.pose.position.y = original_path[i].position.y;
          pose_mod.pose.position.z = original_path[i].position.z;
          pose_mod.pose.orientation.x = q.x();
          pose_mod.pose.orientation.y = q.y();
          pose_mod.pose.orientation.z = q.z();
          pose_mod.pose.orientation.w = q.w();
          local_exploration_path_.poses.push_back(pose_mod);
        }

        visualization_server_->visualizeExplorationPath(
            local_exploration_path_);
      }
    }

    if (!local_exploration_path_.poses.empty()) {
      path_status = GraphManager::GraphStatus::OK;
      ROS_DEBUG("Local path -> OK \n");
    }
  } else {
    ROS_WARN("SG --> No shortest paths to local frontier vertices \n");
    return path_status;
  }

  return path_status;

}  // planLocalPath

void GraphManager::updateGlobalGraph() {
  Vertex* vertex = new Vertex(global_graph_->generateVertexID(), robotState_);
  global_graph_->addVertex(vertex);
  if (prev_pose_vertex_ != nullptr) {
    vertex->parent = prev_pose_vertex_;
    prev_pose_vertex_->children.push_back(vertex);
    double edge_weight =
        (prev_pose_vertex_->state_3d - vertex->state_3d).norm();
    MapManager::VoxelStatus vs = map_manager_->getPathStatus(
        vertex->state.head(3), prev_pose_vertex_->state.head(3),
        robot_box_size_, true);
    if (edge_weight < 2 * planning_params_.nearest_range and
        MapManager::VoxelStatus::kOccupied != vs) {
      global_graph_->addEdge(vertex, prev_pose_vertex_, edge_weight);
    }
  }
  prev_pose_vertex_ = vertex;

  // Add extra edges from pose vertex to densify the graph.

  std::vector<Vertex*> nearby_vertices;

  global_graph_->getNearestVertices(
      &vertex->state, planning_params_.nearest_range, &nearby_vertices);
  for (int i = 0; i < nearby_vertices.size(); ++i) {
    Vertex* neighbour = nearby_vertices[i];
    int num_edges = vertex->edges;
    double weight = (vertex->state - neighbour->state).norm();

    if (num_edges < planning_params_.num_connections_max and
        weight < planning_params_.edge_length_max) {
      if (MapManager::VoxelStatus::kFree ==
          map_manager_->getPathStatus(vertex->state_3d, neighbour->state_3d,
                                      robot_box_size_, true)) {
        global_graph_->addEdge(vertex, neighbour, weight);
        vertex->edges++;
        // neighbour->edges++;
      }
    }
  }

  visualization_server_->visualizeGlobalGraph(global_graph_);

  // update the local sub graph seen from robot's current position to handle
  // dynamic change at global graph level.

  /*
  nearby_vertices.clear();
  global_graph_->getNearestVertices(&robotState_, planning_params_.local_space,
  &nearby_vertices); std::vector<int> violated_vertices;
  violated_vertices.clear();

  for (int i = 0; i < nearby_vertices.size(); i++)
  {

    Vertex* vertex1 = nearby_vertices[i];
    std::vector<std::pair<int, double>> edge_pairs =
  global_graph_->edge_map_[vertex1->id]; bool found_violated_edge = false; for
  (std::pair<int, double> edge_pair : edge_pairs) {

      Vertex* vertex2 = global_graph_->vertices_map_[edge_pair.first];
      MapManager::VoxelStatus vn =
  map_manager_->getPathStatus(vertex1->state.head(3), vertex2->state.head(3),
  robot_box_size_, false);

      // remove the violated edge

      if (MapManager::VoxelStatus::kOccupied == vn) {

        global_graph_->removeEdge(vertex1, vertex2);
        vertex1->edges--;
        vertex2->edges--;
        violated_vertices.push_back(vertex2->id);
      }
      // violated_vertices.push_back(vertex1->id);
    }
  }
  */

  // Try to reconnected additional edges from the violated_vertices. only for
  // debug.. VERY COSTLY. @TODO find a better way to do this reconnections.

  /*
  for (int id : violated_vertices) {

    Vertex * vertex = global_graph_->vertices_map_[id];
    std::vector<Vertex *> nearby_vertices;

    global_graph_->getNearestVertices(&vertex->state,
  planning_params_.nearest_range, &nearby_vertices); for (int i = 0; i <
  nearby_vertices.size(); ++i)
    {
      Vertex* neighbour = nearby_vertices[i];
      int num_edges = vertex->edges;
      double weight = (vertex->state - neighbour->state).norm();

      if (MapManager::VoxelStatus::kFree ==
  map_manager_->getPathStatus(vertex->state.head(3), neighbour->state.head(3),
  robot_box_size_, true))
      {
        global_graph_->addEdge(vertex, neighbour, weight);
      }
    }
  }
  */

  // visualization_server_->visualizeGlobalGraph(global_graph_);
}

void GraphManager::expandGlobalGraph(
    std::vector<std::vector<Vertex*>> dijkstra_paths) {
  GraphManager::updateGlobalGraph();

  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  bool graph_status = true;

  // global_graph_->getNearestVertex(&robotState_, &prev_pose_vertex_);
  if (!dijkstra_paths.empty()) {
    Vertex* prev_vertex = nullptr;
    std::vector<Vertex*> newly_added_vertices;
    newly_added_vertices.clear();
    for (int i = 0; i < dijkstra_paths.size(); ++i) {
      if (i == 0) {
        std::vector<Vertex*> path = dijkstra_paths[i];

        for (int j = 0; j < path.size(); ++j) {
          if (!path[j]->is_evaluated) {
            Vertex* new_vertex =
                new Vertex(global_graph_->generateVertexID(), path[j]->state);
            new_vertex->vol_gain = path[j]->vol_gain;
            global_graph_->addVertex(new_vertex);
            if (new_vertex->vol_gain.is_frontier) {
              global_graph_->frontier_vertices_ids_.push_back(new_vertex->id);
            }
            newly_added_vertices.push_back(new_vertex);
            path[j]->is_evaluated = true;
            path[j]->sub_id = new_vertex->id;

            if (j == 0) {
              new_vertex->parent = prev_pose_vertex_;
              prev_pose_vertex_->children.push_back(new_vertex);
              double edge_weight =
                  (prev_pose_vertex_->state_3d - new_vertex->state_3d).norm();
              global_graph_->addEdge(new_vertex, prev_pose_vertex_,
                                     edge_weight);
              prev_vertex = new_vertex;
            } else {
              new_vertex->parent = prev_vertex;
              prev_vertex->children.push_back(new_vertex);
              double edge_weight =
                  (prev_vertex->state_3d - new_vertex->state_3d).norm();
              global_graph_->addEdge(new_vertex, prev_vertex, edge_weight);
              prev_vertex = new_vertex;
            }
          }
        }
      }

      else {
        std::vector<Vertex*> path = dijkstra_paths[i];

        for (int j = 0; j < path.size(); ++j) {
          if (path[j]->is_evaluated) {
            int evaluated_vert_id = path[j]->sub_id;
            prev_vertex = global_graph_->vertices_map_[evaluated_vert_id];
          }

          if (!path[j]->is_evaluated) {
            Vertex* new_vertex =
                new Vertex(global_graph_->generateVertexID(), path[j]->state);
            new_vertex->vol_gain = path[j]->vol_gain;
            global_graph_->addVertex(new_vertex);
            if (new_vertex->vol_gain.is_frontier) {
              global_graph_->frontier_vertices_ids_.push_back(new_vertex->id);
            }
            newly_added_vertices.push_back(new_vertex);
            path[j]->is_evaluated = true;
            path[j]->sub_id = new_vertex->id;

            new_vertex->parent = prev_vertex;
            prev_vertex->children.push_back(new_vertex);
            double edge_weight =
                (prev_vertex->state_3d - new_vertex->state_3d).norm();
            global_graph_->addEdge(new_vertex, prev_vertex, edge_weight);
            prev_vertex = new_vertex;
          }
        }
      }
    }

    for (Vertex* new_vertex : newly_added_vertices) {
      if (new_vertex->children.empty()) {
        bool sampled = false;
        int temp_it = 0;
        StateVec vertex_state;
        while (temp_it < 100) {
          if (GraphManager::sampleStateToExpandGraph(new_vertex->state,
                                                     vertex_state)) {
            sampled_states_.push_back(vertex_state);
          }
          Vertex* temp_vertex = new Vertex(0, vertex_state);
          GraphManager::computeVolumetricGainRayCasting(vertex_state,
                                                        temp_vertex->vol_gain);

          if (temp_vertex->vol_gain.is_frontier) {
            double weight =
                (vertex_state.head(3) - new_vertex->state.head(3)).norm();
            if (MapManager::VoxelStatus::kOccupied !=
                map_manager_->getPathStatus(vertex_state.head(3),
                                            new_vertex->state_3d,
                                            robot_box_size_, false)) {
              Vertex* new_sampled_vertex =
                  new Vertex(global_graph_->generateVertexID(), vertex_state);
              new_sampled_vertex->vol_gain = temp_vertex->vol_gain;
              new_sampled_vertex->parent = new_vertex;
              new_vertex->children.push_back(new_sampled_vertex);
              global_graph_->addVertex(new_sampled_vertex);
              global_graph_->frontier_vertices_ids_.push_back(
                  new_sampled_vertex->id);
              new_sampled_vertex->parent->vol_gain.is_frontier = false;
              global_graph_->addEdge(new_vertex, new_sampled_vertex, weight);
              break;
            }
          }

          ++temp_it;
        }
      }

      // Get nearby vertices from the current vertex and connect the them to
      // form local edges

      std::vector<Vertex*> nearby_vertices;
      global_graph_->getNearestVertices(&new_vertex->state,
                                        2 * planning_params_.nearest_range,
                                        &nearby_vertices);

      for (int i = 0; i < nearby_vertices.size(); ++i) {
        Vertex* neighbour = nearby_vertices[i];
        int num_edges = new_vertex->edges;
        double weight = (new_vertex->state - neighbour->state).norm();

        if (num_edges < planning_params_.num_connections_max and
            weight < planning_params_.edge_length_max) {
          if (MapManager::VoxelStatus::kOccupied !=
              map_manager_->getPathStatus(new_vertex->state_3d,
                                          neighbour->state_3d, robot_box_size_,
                                          true)) {
            global_graph_->addEdge(new_vertex, neighbour, weight);
            new_vertex->edges++;
            // neighbour->edges++;
          }
        }
      }
    }
  }

  // update the peviously added frontier vertices in updated global graph

  for (std::list<int>::iterator i =
           global_graph_->frontier_vertices_ids_.begin();
       i != global_graph_->frontier_vertices_ids_.end(); i++) {
    Vertex* vertex = global_graph_->vertices_map_[*i];
    // GraphManager::computeVolumetricGainRayCasting(vertex->state,
    // vertex->vol_gain);

    // only update frontier status of vertices within local space (sensor range)
    if ((vertex->state.head(3) - robotState_.head(3)).norm() <
        planning_params_.local_space) {
      GraphManager::computeVolumetricGainRayCasting(vertex->state,
                                                    vertex->vol_gain);
    }
  }

  // remove vertices ids from frontier list if they are no longer frontiers.

  global_graph_->updateFrontierVertices();

  // careful !! visualizeVoxelGain only for debug purpose!!
  // it will tank the computation time for whole planner as atm it is poorly
  // formulated.

  // visualization_server_->visualizeVoxelGain(global_graph_);

  global_root_vertex = prev_pose_vertex_;

  // run dijkstra shortest paths on updated global graph

  global_graph_rep_.reset();
  if (!global_graph_->findShortestPaths(global_root_vertex->id,
                                        global_graph_rep_)) {
    ROS_WARN("Could not find dijkstra shortest paths on the global graph \n");
  }

  std::vector<std::vector<Vertex*>> global_dijkstra_paths;

  global_dijkstra_paths.clear();

  for (int id : global_graph_->frontier_vertices_ids_) {
    Vertex* vertex = global_graph_->vertices_map_[id];

    // if (vertex->vol_gain.is_frontier and vertex->is_cluster_centroid)
    if (vertex->vol_gain.is_frontier) {
      std::vector<Vertex*> global_dijkstra_path;
      global_dijkstra_path.clear();

      global_graph_->getShortestPath(vertex->id, global_graph_rep_, true,
                                     global_dijkstra_path);

      if (!global_dijkstra_path.empty() and
          getPathLength(global_dijkstra_path) > 4) {
        global_dijkstra_paths.push_back(global_dijkstra_path);
      }
    }
  }

  global_paths_.clear();

  if (!global_dijkstra_paths.empty()) {
    for (std::vector<Vertex*> global_dijkstra_path : global_dijkstra_paths) {
      if (!global_dijkstra_path.empty()) {
        Eigen::Vector3d path_goal(global_dijkstra_path.back()->state[0],
                                  global_dijkstra_path.back()->state[1],
                                  global_dijkstra_path.back()->state[2]);

        std::pair<std::vector<Vertex*>, double> p = std::make_pair(
            global_dijkstra_path, getPathLength(global_dijkstra_path));

        global_paths_.push_back(p);
      }
    }
  } else {
    ROS_WARN_STREAM("GG --> No dijkstra paths :( \n");
  }

  visualization_server_->visualizeGlobalPaths(global_paths_);
  visualization_server_->visualizeGlobalGraph(global_graph_);

  // update path to return to base upon every global graph update --> mainly to
  // find shortest route to base station.

  homing_dijkstra_path_.clear();
  global_graph_->getShortestPath(1, global_graph_rep_, true,
                                 homing_dijkstra_path_);

  homing_path_.poses.clear();
  if (!homing_dijkstra_path_.empty()) {
    geometry_msgs::PoseStamped homing_path_pose;
    for (int i = 0; i < homing_dijkstra_path_.size(); i++) {
      homing_path_pose.pose.position.x = homing_dijkstra_path_[i]->state[0];
      homing_path_pose.pose.position.y = homing_dijkstra_path_[i]->state[1];
      homing_path_pose.pose.position.z = homing_dijkstra_path_[i]->state[2];
      homing_path_pose.pose.orientation.x = 0;
      homing_path_pose.pose.orientation.y = 0;
      homing_path_pose.pose.orientation.z = 0;
      homing_path_pose.pose.orientation.w = 1;
      homing_path_pose.header.frame_id = world_frame_id;
      homing_path_pose.header.stamp = ros::Time::now();
      homing_path_.poses.push_back(homing_path_pose);
    }
  }
  visualization_server_->visualizeHomingPath(homing_path_);

  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total =
      duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  ROS_DEBUG_STREAM("GG -> update & homing planning time \t "
                   << duration_total.count() << " ms \n");
}

void GraphManager::setHomingPath() {
  path_iterator_ = 0;
  // GraphManager::updateGlobalGraph();
  local_dijkstra_paths_.clear();
  GraphManager::expandGlobalGraph(local_dijkstra_paths_);
  std::vector<geometry_msgs::Pose> original_path;
  std::vector<geometry_msgs::Pose> modified_path;
  original_path.clear();
  modified_path.clear();
  if (homing_path_.poses.empty()) {
    ROS_WARN("Homing path is empty \n");
    return;
  }
  for (int i = 0; i < homing_path_.poses.size(); ++i) {
    original_path.push_back(homing_path_.poses[i].pose);
  }

  improved_homing_path_.poses.clear();
  bool improvement_success =
      GraphManager::improveFreePath(original_path, modified_path, true, false);

  if (improvement_success) {
    geometry_msgs::PoseStamped pose_mod;
    pose_mod.header.frame_id = world_frame_id;
    pose_mod.header.stamp = ros::Time::now();
    improved_homing_path_.header.frame_id = world_frame_id;
    improved_homing_path_.header.stamp = ros::Time::now();

    for (int i = 0; i < modified_path.size(); ++i) {
      float angle;
      if (i == 0) {
        angle = robotState_[3];
      } else {
        angle = atan2(
            modified_path[i].position.y - modified_path[i - 1].position.y,
            modified_path[i].position.x - modified_path[i - 1].position.x);
      }
      Eigen::Vector3d v(0, 0, angle);
      Eigen::Quaternion<float> q = Euler2Quaternion(v);
      pose_mod.pose.position.x = modified_path[i].position.x;
      pose_mod.pose.position.y = modified_path[i].position.y;
      pose_mod.pose.position.z = modified_path[i].position.z;
      pose_mod.pose.orientation.x = q.x();
      pose_mod.pose.orientation.y = q.y();
      pose_mod.pose.orientation.z = q.z();
      pose_mod.pose.orientation.w = q.w();
      improved_homing_path_.poses.push_back(pose_mod);
    }

    visualization_server_->visualizeHomingPath(improved_homing_path_);
  } else {
    ROS_WARN("Failed to improve homing path : Publishing raw path \n");

    geometry_msgs::PoseStamped pose_mod;
    pose_mod.header.frame_id = world_frame_id;
    pose_mod.header.stamp = ros::Time::now();
    improved_homing_path_.header.frame_id = world_frame_id;
    improved_homing_path_.header.stamp = ros::Time::now();

    for (int i = 0; i < original_path.size(); ++i) {
      float angle;
      if (i == 0) {
        angle = robotState_[3];
      } else {
        angle = atan2(
            original_path[i].position.y - original_path[i - 1].position.y,
            original_path[i].position.x - original_path[i - 1].position.x);
      }
      Eigen::Vector3d v(0, 0, angle);
      Eigen::Quaternion<float> q = Euler2Quaternion(v);
      pose_mod.pose.position.x = original_path[i].position.x;
      pose_mod.pose.position.y = original_path[i].position.y;
      pose_mod.pose.position.z = original_path[i].position.z;
      pose_mod.pose.orientation.x = q.x();
      pose_mod.pose.orientation.y = q.y();
      pose_mod.pose.orientation.z = q.z();
      pose_mod.pose.orientation.w = q.w();
      improved_homing_path_.poses.push_back(pose_mod);
    }

    visualization_server_->visualizeHomingPath(improved_homing_path_);
    // return;
  }
}

nav_msgs::Path GraphManager::getHomingPath() {
  GraphManager::setHomingPath();
  return improved_homing_path_;
}

bool GraphManager::planGlobalPath() {
  bool path_status = true;
  global_exploration_path_.poses.clear();
  std::vector<std::pair<std::vector<Vertex*>, double>> paths_unordered;
  paths_unordered.clear();

  if (!global_paths_.empty()) {
    visualization_server_->visualizeGlobalPaths(global_paths_);

    double gain, path_length, reward;
    double my_reward = -std::numeric_limits<double>::max();

    for (auto i = 0; i < global_paths_.size(); i++) {
      gain = 100 * global_paths_[i].first.back()->vol_gain.unknown_percentage;

      path_length = global_paths_[i].second;

      Eigen::Vector3d p_goal(global_paths_[i].first.back()->state[0],
                             global_paths_[i].first.back()->state[1],
                             global_paths_[i].first.back()->state[2]);
      reward = 1 / (path_length + 1);

      std::pair<std::vector<Vertex*>, double> path_reward_pair =
          std::make_pair(global_paths_[i].first, reward);
      paths_unordered.push_back(path_reward_pair);
    }

    // sort the paths in descending order based on the leaf vertices exploration
    // reward. The lambda expression to compare reward

    std::sort(paths_unordered.begin(), paths_unordered.end(),
              [](const std::pair<std::vector<Vertex*>, double>& a,
                 const std::pair<std::vector<Vertex*>, double>& b) {
                return a.second > b.second;
              });

    // ROS_DEBUG_STREAM ("first and last element reward << " <<
    // paths_unordered.front().second << " , " << paths_unordered.back().second
    // << "\n");

    for (int j = 0; j < paths_unordered.size(); j++) {
      std::vector<Vertex*> path = paths_unordered[j].first;
      best_path_.clear();
      best_path_ = path;
      // GraphManager::improvePath(path, best_path_);
      // if (best_path_.empty()) {
      //   best_path_ = path;
      // }
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

      bool improvement_success = GraphManager::improveFreePath(
          original_path, modified_path, true, false);

      if (improvement_success) {
        geometry_msgs::PoseStamped pose_mod;
        pose_mod.header.frame_id = world_frame_id;
        pose_mod.header.stamp = ros::Time::now();
        global_exploration_path_.header.frame_id = world_frame_id;
        global_exploration_path_.header.stamp = ros::Time::now();

        for (int i = 0; i < modified_path.size(); ++i) {
          float angle;
          if (i == 0) {
            angle = robotState_[3];
          } else {
            angle = atan2(
                modified_path[i].position.y - modified_path[i - 1].position.y,
                modified_path[i].position.x - modified_path[i - 1].position.x);
          }
          Eigen::Vector3d v(0, 0, angle);
          Eigen::Quaternion<float> q = Euler2Quaternion(v);
          pose_mod.pose.position.x = modified_path[i].position.x;
          pose_mod.pose.position.y = modified_path[i].position.y;
          pose_mod.pose.position.z = modified_path[i].position.z;
          pose_mod.pose.orientation.x = q.x();
          pose_mod.pose.orientation.y = q.y();
          pose_mod.pose.orientation.z = q.z();
          pose_mod.pose.orientation.w = q.w();
          global_exploration_path_.poses.push_back(pose_mod);
        }
        visualization_server_->visualizeExplorationPath(
            global_exploration_path_);
        break;
      } else if (j == (paths_unordered.size() - 1)) {
        ROS_WARN("GG -> Improved global path is empty \n");
        geometry_msgs::PoseStamped pose_mod;
        pose_mod.header.frame_id = world_frame_id;
        pose_mod.header.stamp = ros::Time::now();
        global_exploration_path_.header.frame_id = world_frame_id;
        global_exploration_path_.header.stamp = ros::Time::now();

        for (int i = 0; i < original_path.size(); ++i) {
          float angle;
          if (i == 0) {
            angle = robotState_[3];
          } else {
            angle = atan2(
                original_path[i].position.y - original_path[i - 1].position.y,
                original_path[i].position.x - original_path[i - 1].position.x);
          }
          Eigen::Vector3d v(0, 0, angle);
          Eigen::Quaternion<float> q = Euler2Quaternion(v);
          pose_mod.pose.position.x = original_path[i].position.x;
          pose_mod.pose.position.y = original_path[i].position.y;
          pose_mod.pose.position.z = original_path[i].position.z;
          pose_mod.pose.orientation.x = q.x();
          pose_mod.pose.orientation.y = q.y();
          pose_mod.pose.orientation.z = q.z();
          pose_mod.pose.orientation.w = q.w();
          global_exploration_path_.poses.push_back(pose_mod);
        }

        visualization_server_->visualizeExplorationPath(
            global_exploration_path_);
      }
    }
  } else {
    path_status = false;
    ROS_WARN("GG : No shortest paths to frontiers \n");
    return path_status;
  }

  return path_status;

}  // planGlobalPaths

GraphManager::GraphStatus GraphManager::runGlobalPlanner() {
  GraphManager::GraphStatus global_planner_status =
      GraphManager::GraphStatus::NOT_OK;

  GraphManager::expandGlobalGraph(local_dijkstra_paths_);

  if (GraphManager::planGlobalPath()) {
    global_planner_status = GraphManager::GraphStatus::OK;
  } else {
    global_planner_status = GraphManager::GraphStatus::ERR_NO_FEASIBLE_PATH;
  }

  return global_planner_status;
}

void GraphManager::setExplorationDirection(geometry_msgs::PoseStamped pose) {
  exploration_dir_pose = pose;
}

double GraphManager::getRelativeAngle(std::vector<Vertex*> exp_path) {
  std::vector<Eigen::Vector3d> path;
  path.clear();
  for (int i = 0; i < exp_path.size(); ++i) {
    path.push_back(exp_path[i]->state.head(3));
  }

  double yaw = Trajectory::estimateDirectionFromPath(path);

  std::vector<Eigen::Vector3d> state_hist_path;
  state_hist_path.clear();
  int pose_graph_vertices = pose_graph_->getNumVertices();
  if (pose_graph_vertices > 1) {
    Eigen::Vector3d v1 =
        pose_graph_->vertices_map_[pose_graph_vertices - 1]->state.head(3);
    Eigen::Vector3d v0 =
        pose_graph_->vertices_map_[pose_graph_vertices - 2]->state.head(3);
    state_hist_path.push_back(v0);
    state_hist_path.push_back(v1);
  }

  double state_hist_yaw =
      Trajectory::estimateDirectionFromPath(state_hist_path);

  Eigen::Vector3d v_path(0, 0, yaw);
  Eigen::Quaternion<float> q2 = Euler2Quaternion(v_path);
  Eigen::Vector3d v_state(0, 0, state_hist_yaw);
  Eigen::Quaternion<float> q1 = Euler2Quaternion(v_state);

  exploration_dir_pose.header.frame_id = world_frame_id;
  exploration_dir_pose.header.stamp = ros::Time::now();
  exploration_dir_pose.pose.position.x = robotState_[0];
  exploration_dir_pose.pose.position.y = robotState_[1];
  exploration_dir_pose.pose.position.z = robotState_[2];

  exploration_dir_pose.pose.orientation.x = q1.x();
  exploration_dir_pose.pose.orientation.y = q1.y();
  exploration_dir_pose.pose.orientation.z = q1.z();
  exploration_dir_pose.pose.orientation.w = q1.w();
  visualization_server_->visualizeExplorationDirection(exploration_dir_pose);

  double angle = std::acos(q1.dot(q2)) * 2.0 * 180 / M_PI;

  return angle;
}

/*

TODO:

  1. Homing planner to be running in the background -- done

  2. Rosservice to fetch new exploration path -- done

  3. Convert path to trajectory in stage_interface. -- done

  4. Add initialization behaviour in stage_interface. -- done

  3. Rosservice to fetch return to home path -- done

  4. Rosnode health monitoring ????

*/
