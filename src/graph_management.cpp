#include "graph_management.h"

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

GraphManager::GraphManager(ros::NodeHandle private_nh_)
    : m_nh(), m_worldFrameId("world"), m_odometry("spot/ground_truth"),
      m_currentHeading(0), m_local_space(10), m_local_fov(40),
      m_ufomap("/ufomap_mapping_server_node/map"), m_max_allowed_Z(4),
      m_obb_radius(0.5), m_planning_depth(0), m_volumetric_gain_threashold(10),
      m_min_dist_bw_nodes(0.8), m_iterations_local(2000), m_iterations_global(6000),
      m_max_edge_length(4)

{

  ros::NodeHandle private_nh = private_nh_;
  // Set variables
  private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
  private_nh.param("odometry", m_odometry, m_odometry);

  private_nh.param("local_sliding_space", m_local_space, m_local_space);
  private_nh.param("local_field_of_view", m_local_fov, m_local_space);
  private_nh.param("ufomap", m_ufomap, m_ufomap);
  private_nh.param("safety_height", m_max_allowed_Z, m_max_allowed_Z);
  private_nh.param("traversability_radius", m_obb_radius, m_obb_radius);
  private_nh.param("planning_check_depth", m_planning_depth,
                   m_planning_depth);
  private_nh.param("volumetric_gain_threashold", m_volumetric_gain_threashold,
                   m_volumetric_gain_threashold);
  private_nh.param("graph_density_coefficient", m_min_dist_bw_nodes,
                   m_min_dist_bw_nodes);
  private_nh.param("maximum_edge_length", m_max_edge_length, m_max_edge_length);
  private_nh.param("maximum_iterations_local", m_iterations_local, m_iterations_local);
  private_nh.param("maximum_iterations_global", m_iterations_global, m_iterations_global);

  // Register to the subscriber topic and register it to the callback function

  m_poseSubscriber =
      new message_filters::Subscriber<nav_msgs::Odometry>(m_nh, m_odometry, 5);
  m_poseSubscriber->registerCallback(
      boost::bind(&GraphManager::updateCurrentPosition, this, _1));

  m_ufoMapSubscriber =
      new message_filters::Subscriber<ufomap_msgs::UFOMapStamped>(m_nh,
                                                                  m_ufomap, 1);
  m_ufoMapSubscriber->registerCallback(
      boost::bind(&GraphManager::mapCallback, this, _1));

  // Register publishers
  m_frontierPub = m_nh.advertise<visualization_msgs::MarkerArray>(
      "/frontier_nodes_vis", 1, true);

  /* Ros services */
  m_frontier_client = m_nh.serviceClient<stage_planner::frontierRequest>(
      "request_frontier_target");
  
  m_target_query_client = m_nh.serviceClient<stage_planner::isFrontier>(
      "query_frontier_target");

  
  m_exploration_path_srv = m_nh.advertiseService("next_exploration_path", &GraphManager::nextExplorationPath, this);


};

ufo::map::OccupancyMapColor m_ufoMAP(0.2);

void GraphManager::mapCallback(
    ufomap_msgs::UFOMapStamped::ConstPtr const &msg)
{

  if (ufomap_msgs::msgToUfo(msg->map, m_ufoMAP))
  {
    // ROS_INFO("MAP RECEIVED");
    map_received = true;
  }
  else
  {
    ROS_WARN("MAP NOT RECEIVED");
  }
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

void GraphManager::updateCurrentPosition(
    const nav_msgs::Odometry::ConstPtr &pose)
{

  if (firstTime)
  {
    home_position.x() = pose->pose.pose.position.x;
    home_position.y() = pose->pose.pose.position.y;
    home_position.z() = pose->pose.pose.position.z;
    m_prevPose = *pose;
    firstTime = false;
  }
  else
  {
    m_currentPose = *pose;

    ufo::math::Vector3 newPoint(pose->pose.pose.position.x, pose->pose.pose.position.y,
                                pose->pose.pose.position.z);
    ufo::math::Vector3 oldPoint(m_prevPose.pose.pose.position.x, m_prevPose.pose.pose.position.y,
                                m_prevPose.pose.pose.position.z);
    geometry_msgs::PoseWithCovariance old_pose = m_prevPose.pose;
    geometry_msgs::PoseWithCovariance new_pose = m_currentPose.pose;
    if (GraphManager::calculateDistance(newPoint, oldPoint) > 1)
    {
      m_prevPose = *pose;
      PoseGraph::pg_node previous_node(old_pose);
      PoseGraph::pg_node new_node(new_pose);
      // new_node.add_sensor_measurement(float current_measurement);
      poseGraph.add_pg_node(previous_node);
      poseGraph.add_pg_node(new_node);
      poseGraph.connectNodes(previous_node, new_node);
      
      // Plan homing path in the background every few meters the robot moves to assure an updated homing path at all time. 

      // if (!global_graph_container_.empty()) {
      //   planHomingPath ();
      // }
      
    }
  }

  m_currentPosition.x() = m_currentPose.pose.pose.position.x;
  m_currentPosition.y() = m_currentPose.pose.pose.position.y;
  m_currentPosition.z() = m_currentPose.pose.pose.position.z;

  tf::Quaternion q(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y,
                   pose->pose.pose.orientation.z,
                   pose->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  m_currentHeading = yaw * (180 / 3.14159);
}

ufo::math::Vector3 GraphManager::fetchCurrentPosition()
{
  return m_currentPosition;
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

double GraphManager::calculateAngle(ufo::math::Vector3 fpoint,
                                    ufo::math::Vector3 position,
                                    double heading)
{

  double head =
      atan2((fpoint.y() - position.y()), (fpoint.x() - position.x())) *
      (180 / 3.14159);
  double angle = head - heading;
  if (head > -180 && head < -90)
  {
    if (heading > -180 && heading < -90)
    {
      angle = head - heading;
      if (angle > 180)
      {
        angle = angle - 360;
      }
    }
    else
    {
      angle = angle + 360;
      if (angle > 180)
      {
        angle = angle - 360;
      }
    }
  }
  else if (head > 0)
  {
    if (heading > -180 && heading < -90)
    {
      angle = angle - 360;
      if (angle > 180)
      {
        angle = angle - 360;
      }
    }
    else
    {
      angle = angle;
      if (angle > 180)
      {
        angle = angle - 360;
      }
    }
  }
  return angle;
}

bool GraphManager::isLocal(ufo::math::Vector3 point)
{

  double angle =
      GraphManager::calculateAngle(point, m_currentPosition, m_currentHeading);
  double distance = GraphManager::calculateDistance(point, m_currentPosition);

  if ((angle < m_local_fov && angle > m_local_fov) && distance < m_local_space)
  {
    return true;
  }
  else
  {
    return false;
  }
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
GraphManager::tunePlanningSpace(float range, ufo::math::Vector3 root_node,
                                ufo::map::OccupancyMapColor const &map)
{

  highest_x = -std::numeric_limits<float>::max();
  highest_y = -std::numeric_limits<float>::max();
  highest_z = -std::numeric_limits<float>::max();
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();

  ufo::math::Vector3 minPoint(root_node.x() - 1 * range,
                              root_node.y() - 1 * range,
                              root_node.z() - 1 * range);
  ufo::math::Vector3 maxPoint(root_node.x() + 1 * range,
                              root_node.y() + 1 * range,
                              root_node.z() + 1 * range);
  ufo::geometry::AABB aabb(minPoint, maxPoint);

  for (auto it = map.beginLeaves(aabb, false, true, false, false,
                                 m_planning_depth);
       it != map.endLeaves(); ++it)
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

ufo::math::Vector3 *GraphManager::node::getNodePointer() { return point; }

void GraphManager::node::addChild(node *newChild)
{
  myChilds.push_back(newChild);
}

void GraphManager::node::addParent(node *newParent) { myParent = newParent; }

void GraphManager::node::setTraversabilityStatus(bool Traversable)
{
  isTraversableByDrone = Traversable;
}
bool GraphManager::node::getTraversabilityStatus()
{
  return isTraversableByDrone;
}
void GraphManager::node::readyForDeletion() { delete point; }

std::vector<GraphManager::node *> GraphManager::sampleNodes(ufo::math::Vector3 root_node,
                                                            int max_iterations,
                                                            float range,
                                                            ufo::map::OccupancyMapColor const &map)
{ 
  planning_iteration_++;
  std::vector<node *> temporary_nodes;
/*
  for (std::vector<node *>::iterator it_clear_helper = temporary_nodes.begin();
       it_clear_helper != --temporary_nodes.end(); it_clear_helper++)
  {
    (*it_clear_helper)->readyForDeletion();
    delete (*it_clear_helper);
  }
*/
  // temporary_nodes.clear();
  node *current_pos = new node(root_node.x(), root_node.y(), root_node.z());
  current_pos->addParent(nullptr);
  temporary_nodes.push_back(current_pos);

  srand(time(0));
  int itterations = 0;

  while (itterations < max_iterations)
  {
    // Generate a random point

    ufo::math::Vector3 box = tunePlanningSpace(range, root_node, map);

    float x = lowest_x + abs(1024 * rand() / (RAND_MAX + 1.0)) * box.x();
    float y = lowest_y + abs(1024 * rand() / (RAND_MAX + 1.0)) * box.y();
    float z = lowest_z + abs(1024 * rand() / (RAND_MAX + 1.0)) * box.z();
    ufo::math::Vector3 random_point(x, y, z);
    ufo::geometry::Sphere point_sphere(random_point, m_obb_radius);
    if (!isInCollision(map, point_sphere, true, false, true,
                       m_planning_depth) and
        isInCollision(map, point_sphere, false, true, false,
                      m_planning_depth))
    {
      float distance = std::numeric_limits<float>::max();
      node *parent;
      std::vector<node *>::iterator it_node;
      for (it_node = temporary_nodes.begin(); it_node != temporary_nodes.end();
           it_node++)
      {
        ufo::math::Vector3 direction = random_point - *((*it_node)->point);
        double new_distance = abs(direction.norm());
        if (new_distance < distance)
        {
          distance = new_distance;
          parent = *it_node;
        }
      };
      ufo::math::Vector3 start_point(parent->point->x(), parent->point->y(),
                                     parent->point->z());
      ufo::math::Vector3 direction = random_point - start_point;
      ufo::geometry::LineSegment myline(random_point, start_point);

      if (!isInCollision(map, myline, true, false, true,
                         m_planning_depth) and
          calculateDistance(random_point, start_point) > m_min_dist_bw_nodes)

      {
        node *new_node = new node(x, y, z);
        new_node->addParent(parent);
        parent->addChild(new_node);
        temporary_nodes.push_back(new_node);
      }
    };
    itterations++;
  };
    
    ROS_INFO_STREAM("Planning space contains --> " << temporary_nodes.size()
                                                   << " nodes\n");

    PoseGraph::planning_instance planning_inst(planning_iteration_);
    planning_inst.retracing = false;
    planning_inst.planning_iteration = planning_iteration_;
    planning_inst.reserved_samples = temporary_nodes;

    poseGraph.planning_instances.push_back(planning_inst);

    // Start using reserved nodes to build and manage a global graph.

    // if (planning_iteration_ > 1) {
    //   
    //   ROS_INFO("Global graph construction");
    //
    //   GraphManager::constructGlobalGraph ();
    // }

  return temporary_nodes;
}

std::vector<GraphManager::Graph::Point> GraphManager::Graph::fetchPath(Graph::Point &start_pos, Graph::Point &end_pos,
                                                                       std::vector<Graph::vertex> all_vertices)
{
  vertex start_vert;
  vertex end_vert;

  for (vertex &vert : all_vertices)
  {
    if (vert.location == end_pos)
      start_vert = vert;
    if (vert.location == start_pos)
      end_vert = vert;
  }

  std::vector<Point> found_path; // Output path
  std::set<vertex> open_list;
  std::vector<Point> closed_list;

  open_list.emplace(start_vert);

  while (!open_list.empty())
  {
    vertex current_vert = *open_list.begin();

    open_list.erase(open_list.begin());
    closed_list.push_back(current_vert.location);

    if (current_vert.location == end_vert.location)
    {
      vertex *parent = &current_vert;

      while (parent != nullptr)
      {
        found_path.push_back(parent->location);
        parent = parent->parent.get();
      }

      // ROS_INFO_STREAM(" found path length in astar function " << found_path.size());
      
      return (found_path);
    }
    
    // cout << "current vert edges : " << (*current_vert.edges).size() << endl;  

    for (vertex &vert : *current_vert.edges)
    {
      vertex child(vert);
      child.parent.reset(new vertex(current_vert));

      if (std::find_if(closed_list.begin(), closed_list.end(),
                       [&child](Point &pt)
                       {
                         return (pt == child.location);
                       }) != closed_list.end())
        continue;

      child.dist_to_start = current_vert.dist_to_start +
                            child.location.DistTo(current_vert.location);

      child.total_dist = (child.dist_to_start +
                                (child.location.DistTo(end_vert.location)));

      if (std::find_if(open_list.begin(), open_list.end(),
                       [&child](const vertex &vert)
                       {
                         return ((vert == child) &&
                                 (child.dist_to_start > vert.dist_to_start));
                       }) != open_list.end())
        continue;

      open_list.emplace(child);
    }
  }

  // ROS_INFO_STREAM(" found path length in astar function " << found_path.size());

  return (found_path);
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

// Maintain a global graph by re-using the nodes created at every planning iteration for local exploration. The global graph only connects nodes from the most recent planning iteration and the one before that. The globalGraph function is called at every planning iteration to maintain sparse graph used for global re-positioning as well as for homing purpose. 

void GraphManager::constructGlobalGraph () {

  // The first set of current and previous iteration nodes construct the base for the global graph. After that for each iteration the nodes are added to the graph and connected to the reserved graph. 

  std::vector<GraphManager::node *> current_iteration_nodes_;
  std::vector<GraphManager::node *> previous_iteration_nodes_;
  std::vector<GraphManager::node *> nodes;

  int temp_var = poseGraph.planning_instances.size();

  current_iteration_nodes_ = poseGraph.planning_instances[temp_var].reserved_samples;

  previous_iteration_nodes_ = poseGraph.planning_instances[temp_var-1].reserved_samples;

  for (auto i : previous_iteration_nodes_) {
    nodes.push_back(i);
  }
  
  for (auto i : current_iteration_nodes_) {
    nodes.push_back(i);
  }

  std::vector<Graph::vertex> global_nodes;

  std::vector<std::pair<std::vector<Graph::vertex>::iterator,
                        std::vector<Graph::vertex>::iterator>> global_nodes_pairs;

  for (auto i = nodes.begin(); i != nodes.end(); i++) {

    global_nodes.push_back( Graph::Point{(*i)->point->x(), (*i)->point->y(), (*i)->point->z()} );

  }

  for (auto i = global_nodes.begin(); i != std::prev(global_nodes.end(), 1); i++)
  {

    ufo::math::Vector3 v1((*i).location.x, (*i).location.y, (*i).location.z);
    
    for (auto i2 = i; i2 != global_nodes.end(); i2++)
    {

      ufo::math::Vector3 v2((*i2).location.x, (*i2).location.y, (*i2).location.z);
      
      double distance = calculateDistance(v1, v2);

      if (distance < m_max_edge_length)
      {

        global_nodes_pairs.push_back(std::make_pair(i, i2));
      }
    }
  }

  ROS_INFO_STREAM ("Global nodes pairs : " << global_nodes_pairs.size());

  std::vector<std::pair<Graph::vertex, Graph::vertex>> vec1, vec2, vec3, vec4;

#pragma omp parallel
  {
#pragma omp single
    {
#pragma omp task
      {

        for (auto i = global_nodes_pairs.begin(); i != std::next(global_nodes_pairs.begin(), global_nodes_pairs.size() / 4); i++)
        {
          ufo::math::Vector3 n1(((*i).first)->location.x,
                                  ((*i).first)->location.y,
                                  ((*i).first)->location.z);
          ufo::math::Vector3 n2(((*i).second)->location.x,
                                  ((*i).second)->location.y,
                                  ((*i).second)->location.z);


          ufo::geometry::OBB obb = makeOBB(n1, n2, m_obb_radius);

          if (!isInCollision(m_ufoMAP, obb, true, false, true, m_planning_depth))
          {
            vec1.push_back(std::make_pair(*i->first, *i->second));
          }
        }
      }
#pragma omp task
      {
        for (auto i = std::next(global_nodes_pairs.begin(), global_nodes_pairs.size() / 4); i != std::next(global_nodes_pairs.begin(), global_nodes_pairs.size() / 2); i++)
        {
          ufo::math::Vector3 n1(((*i).first)->location.x,
                                  ((*i).first)->location.y,
                                  ((*i).first)->location.z);
          ufo::math::Vector3 n2(((*i).second)->location.x,
                                  ((*i).second)->location.y,
                                  ((*i).second)->location.z);


          ufo::geometry::OBB obb = makeOBB(n1, n2, m_obb_radius);

          if (!isInCollision(m_ufoMAP, obb, true, false, true, m_planning_depth))
          {
            vec2.push_back(std::make_pair(*i->first, *i->second));
          }
        }
      }
#pragma omp task
      {
        for (auto i = std::next(global_nodes_pairs.begin(), global_nodes_pairs.size() / 2); i != std::prev(global_nodes_pairs.end(), global_nodes_pairs.size() / 4); i++)
        {
          ufo::math::Vector3 n1(((*i).first)->location.x,
                                  ((*i).first)->location.y,
                                  ((*i).first)->location.z);
          ufo::math::Vector3 n2(((*i).second)->location.x,
                                  ((*i).second)->location.y,
                                  ((*i).second)->location.z);


          ufo::geometry::OBB obb = makeOBB(n1, n2, m_obb_radius);

          if (!isInCollision(m_ufoMAP, obb, true, false, true, m_planning_depth))
          {
            vec3.push_back(std::make_pair(*i->first, *i->second));
          }
        }
      }
#pragma omp task
      {
        for (auto i = std::prev(global_nodes_pairs.end(), global_nodes_pairs.size() / 4); i != global_nodes_pairs.end(); i++)
        {
          ufo::math::Vector3 n1(((*i).first)->location.x,
                                  ((*i).first)->location.y,
                                  ((*i).first)->location.z);
          ufo::math::Vector3 n2(((*i).second)->location.x,
                                  ((*i).second)->location.y,
                                  ((*i).second)->location.z);


          ufo::geometry::OBB obb = makeOBB(n1, n2, m_obb_radius);

          if (!isInCollision(m_ufoMAP, obb, true, false, true, m_planning_depth))
          {
            vec4.push_back(std::make_pair(*i->first, *i->second));
          }
        }
      }
    }
  }
  
    for (auto i = vec1.begin(); i != vec1.end(); i++)
    {

      // if ((i->first).getEdges() < 8 and (i->second).getEdges() < 8) {
      if (!global_graph_container_.empty()) {

      if (std::find(global_graph_container_.begin(), global_graph_container_.end(),
                  i->first) != global_graph_container_.end() 
          && std::find(global_graph_container_.begin(), global_graph_container_.end(),
                  i->second) != global_graph_container_.end()) {

        global_graph_container_.push_back(i->first);
        global_graph_container_.push_back(i->second);

      }
  
    } else {

      global_graph_container_.push_back(i->first);
      global_graph_container_.push_back(i->second);

    }

    (i->first).createEdge(i->second);
    (i->second).createEdge(i->first);
 

  }

     for (auto i = vec2.begin(); i != vec2.end(); i++)
    {

      // if ((i->first).getEdges() < 8 and (i->second).getEdges() < 8) {
      if (!global_graph_container_.empty()) {

      if (std::find(global_graph_container_.begin(), global_graph_container_.end(),
                  i->first) != global_graph_container_.end() 
          && std::find(global_graph_container_.begin(), global_graph_container_.end(),
                  i->second) != global_graph_container_.end()) {

        global_graph_container_.push_back(i->first);
        global_graph_container_.push_back(i->second);

      }
  
    } else {

      global_graph_container_.push_back(i->first);
      global_graph_container_.push_back(i->second);

    }

    (i->first).createEdge(i->second);
    (i->second).createEdge(i->first);
 

  } 

    for (auto i = vec3.begin(); i != vec3.end(); i++)
    {

      // if ((i->first).getEdges() < 8 and (i->second).getEdges() < 8) {
      if (!global_graph_container_.empty()) {

      if (std::find(global_graph_container_.begin(), global_graph_container_.end(),
                  i->first) != global_graph_container_.end() 
          && std::find(global_graph_container_.begin(), global_graph_container_.end(),
                  i->second) != global_graph_container_.end()) {

        global_graph_container_.push_back(i->first);
        global_graph_container_.push_back(i->second);

      }
  
    } else {

      global_graph_container_.push_back(i->first);
      global_graph_container_.push_back(i->second);

    }

    (i->first).createEdge(i->second);
    (i->second).createEdge(i->first);
  

  }

    for (auto i = vec4.begin(); i != vec4.end(); i++)
    {

      // if ((i->first).getEdges() < 8 and (i->second).getEdges() < 8) {
     if (!global_graph_container_.empty()) {

      if (std::find(global_graph_container_.begin(), global_graph_container_.end(),
                  i->first) != global_graph_container_.end() 
          && std::find(global_graph_container_.begin(), global_graph_container_.end(),
                  i->second) != global_graph_container_.end()) {

        global_graph_container_.push_back(i->first);
        global_graph_container_.push_back(i->second);

      }
  
    } else {

      global_graph_container_.push_back(i->first);
      global_graph_container_.push_back(i->second);

    }

    (i->first).createEdge(i->second);
    (i->second).createEdge(i->first);
   

  }

} // constructGlobalGraph 

std::vector<GraphManager::Graph::vertex> GraphManager::constructGraph(int iterations, float planning_space)
{


  high_resolution_clock::time_point start_total = high_resolution_clock::now();

  std::vector<GraphManager::node *> nodes;
  
  if (!run_global_planner) 
  {
    nodes = sampleNodes(m_currentPosition, iterations, planning_space, m_ufoMAP);
  }
  
  // ROS_INFO_STREAM ("nodes : " << nodes.size());
  
  std::vector<Graph::vertex> local_nodes;

  std::vector<std::pair<std::vector<Graph::vertex>::iterator,
                        std::vector<Graph::vertex>::iterator>> local_nodes_pairs;

  for (auto i = nodes.begin(); i != nodes.end(); i++) {

    local_nodes.push_back( Graph::Point{(*i)->point->x(), (*i)->point->y(), (*i)->point->z()} );

  }

  for (auto i = local_nodes.begin(); i != std::prev(local_nodes.end(), 1); i++)
  {

    ufo::math::Vector3 v1((*i).location.x, (*i).location.y, (*i).location.z);
    
    for (auto i2 = i; i2 != local_nodes.end(); i2++)
    {

      ufo::math::Vector3 v2((*i2).location.x, (*i2).location.y, (*i2).location.z);
      
      double distance = calculateDistance(v1, v2);

      if (distance < m_max_edge_length and distance > 2)
      {

        local_nodes_pairs.push_back(std::make_pair(i, i2));
      }
    }
  }

  ROS_INFO_STREAM ("Local nodes pairs : " << local_nodes_pairs.size());

  std::vector<std::pair<Graph::vertex, Graph::vertex>> vec1, vec2, vec3, vec4, my_vec;

#pragma omp parallel
  {
#pragma omp single
    {
#pragma omp task
      {

        for (auto i = local_nodes_pairs.begin(); i != std::next(local_nodes_pairs.begin(), local_nodes_pairs.size() / 4); i++)
        {
          ufo::math::Vector3 n1(((*i).first)->location.x,
                                  ((*i).first)->location.y,
                                  ((*i).first)->location.z);
          ufo::math::Vector3 n2(((*i).second)->location.x,
                                  ((*i).second)->location.y,
                                  ((*i).second)->location.z);


          ufo::geometry::OBB obb = makeOBB(n1, n2, m_obb_radius);

          if (!isInCollision(m_ufoMAP, obb, true, false, true, m_planning_depth))
          {
            vec1.push_back(std::make_pair(*i->first, *i->second));
          }
        }
      }
#pragma omp task
      {
        for (auto i = std::next(local_nodes_pairs.begin(), local_nodes_pairs.size() / 4); i != std::next(local_nodes_pairs.begin(), local_nodes_pairs.size() / 2); i++)
        {
          ufo::math::Vector3 n1(((*i).first)->location.x,
                                  ((*i).first)->location.y,
                                  ((*i).first)->location.z);
          ufo::math::Vector3 n2(((*i).second)->location.x,
                                  ((*i).second)->location.y,
                                  ((*i).second)->location.z);


          ufo::geometry::OBB obb = makeOBB(n1, n2, m_obb_radius);

          if (!isInCollision(m_ufoMAP, obb, true, false, true, m_planning_depth))
          {
            vec2.push_back(std::make_pair(*i->first, *i->second));
          }
        }
      }
#pragma omp task
      {
        for (auto i = std::next(local_nodes_pairs.begin(), local_nodes_pairs.size() / 2); i != std::prev(local_nodes_pairs.end(), local_nodes_pairs.size() / 4); i++)
        {
          ufo::math::Vector3 n1(((*i).first)->location.x,
                                  ((*i).first)->location.y,
                                  ((*i).first)->location.z);
          ufo::math::Vector3 n2(((*i).second)->location.x,
                                  ((*i).second)->location.y,
                                  ((*i).second)->location.z);


          ufo::geometry::OBB obb = makeOBB(n1, n2, m_obb_radius);

          if (!isInCollision(m_ufoMAP, obb, true, false, true, m_planning_depth))
          {
            vec3.push_back(std::make_pair(*i->first, *i->second));
          }
        }
      }
#pragma omp task
      {
        for (auto i = std::prev(local_nodes_pairs.end(), local_nodes_pairs.size() / 4); i != local_nodes_pairs.end(); i++)
        {
          ufo::math::Vector3 n1(((*i).first)->location.x,
                                  ((*i).first)->location.y,
                                  ((*i).first)->location.z);
          ufo::math::Vector3 n2(((*i).second)->location.x,
                                  ((*i).second)->location.y,
                                  ((*i).second)->location.z);


          ufo::geometry::OBB obb = makeOBB(n1, n2, m_obb_radius);

          if (!isInCollision(m_ufoMAP, obb, true, false, true, m_planning_depth))
          {
            vec4.push_back(std::make_pair(*i->first, *i->second));
          }
        }
      }
    }
  }


  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total =
      duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  ROS_INFO_STREAM ( "\033[;34m Graph building takes... \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n ");

    std::vector<Graph::vertex> graph_container;

    for (auto i = vec1.begin(); i != vec1.end(); i++)
    {

      if ((i->first).getEdges() < 8 and (i->second).getEdges() < 8) {
      graph_container.push_back((i)->first);
      graph_container.push_back((i)->second);
      ((i)->first).createEdge((i)->second);
      ((i)->second).createEdge((i)->first);
    

    }

  }

     for (auto i = vec2.begin(); i != vec2.end(); i++)
    {

      if ((i->first).getEdges() < 8 and (i->second).getEdges() < 8) {
      graph_container.push_back((i)->first);
      graph_container.push_back((i)->second);
      ((i)->first).createEdge((i)->second);
      ((i)->second).createEdge((i)->first);
    
    }

  } 

    for (auto i = vec3.begin(); i != vec3.end(); i++)
    {

      if ((i->first).getEdges() < 8 and (i->second).getEdges() < 8) {
      graph_container.push_back((i)->first);
      graph_container.push_back((i)->second);
      ((i)->first).createEdge((i)->second);
      ((i)->second).createEdge((i)->first);
    
    }

  }

    for (auto i = vec4.begin(); i != vec4.end(); i++)
    {

      if ((i->first).getEdges() < 8 and (i->second).getEdges() < 8) {
      graph_container.push_back((i)->first);
      graph_container.push_back((i)->second);
      ((i)->first).createEdge((i)->second);
      ((i)->second).createEdge((i)->first);
    
    }

  }


  return graph_container;

} // constructGraph

std::vector<GraphManager::Graph::Point> GraphManager::planPath()
{

  int iterations;
  double planning_space;
  double reposition_reward = -std::numeric_limits<double>::max();
  double reward;
  fetchExplorationTargets();
  
  // ROS_INFO_STREAM ("Exploration target X " << m_local_exploration_target.x() );

  if (local_exploration)
  {
    iterations = m_iterations_local;
    planning_space = m_local_space;
    exploration_target = m_local_exploration_target;
  }
  else if (global_exploration)
  {
    iterations = m_iterations_global;
    planning_space = m_global_space;

    for (ufo::math::Vector3 v : m_global_repos_targets)
    {

      double volumetric_info_gain = computeVolumetricGain(v, m_ufoMAP);
      double eucl_distance = calculateDistance(v, fetchCurrentPosition());

      reward = volumetric_info_gain / eucl_distance;

      if (reward > reposition_reward)
      {
        reposition_reward = reward;
        exploration_target = v;
      }
    }
  }

  std::vector<GraphManager::Graph::vertex> planning_graph;
  
  std::vector<GraphManager::Graph::vertex> local_graph = constructGraph(iterations, planning_space);
  
  // ROS_INFO_STREAM ("local graph has " << local_graph.size() << " nodes");
  
  if (run_global_planner or global_exploration) {
    planning_graph = global_graph_container_;
  } else {
    planning_graph = local_graph;
  }

  ROS_INFO_STREAM ("planning graph has " << planning_graph.size() << " nodes");

  double start_x, start_y, start_z, end_x, end_y, end_z;

    double close_dis = std::numeric_limits<double>::max();
    double end_dis = std::numeric_limits<double>::max();
    double my_end_dis = std::numeric_limits<double>::max();
    double my_close_dis = std::numeric_limits<double>::max();

  for (auto v = planning_graph.begin(); v != planning_graph.end(); v++)
  {

      close_dis = std::sqrt(
          std::pow((*v).location.x - fetchCurrentPosition().x(), 2) +
          std::pow((*v).location.y - fetchCurrentPosition().y(), 2) +
          std::pow((*v).location.z - fetchCurrentPosition().z(), 2));

      end_dis = std::sqrt(std::pow((*v).location.x -
                                       exploration_target.x(),
                                   2) +
                          std::pow((*v).location.y -
                                       exploration_target.y(),
                                   2) +
                          std::pow((*v).location.z -
                                       exploration_target.z(),
                                   2));

      if (close_dis < my_close_dis)
      {
        my_close_dis = close_dis;
        start_x = ((*v).location.x);
        start_y = ((*v).location.y);
        start_z = ((*v).location.z);
      }

      if (end_dis < my_end_dis)
      {
        my_end_dis = end_dis;
        end_x = ((*v).location.x);
        end_y = ((*v).location.y);
        end_z = ((*v).location.z);
      }
  }
  
  ROS_INFO_STREAM ("start " << start_x << " " << start_y << " " << start_z );
  ROS_INFO_STREAM ("end   " << end_x << " " << end_y << " " << end_z );

  Graph::Point start_pos {start_x, start_y, start_z};
  Graph::Point end_pos {end_x, end_y, end_z};

  Graph graph;
  
  Astar_path = graph.fetchPath(start_pos, end_pos, planning_graph);

  if (Astar_path.size() < 2) {

  planPath();

  }

  ROS_INFO_STREAM ("A-star path size : " << Astar_path.size());
  
  return Astar_path;
}

nav_msgs::Path GraphManager::fetchExplorationPath()
{

  auto path = planPath();

  std::vector<Graph::vertex> improvement_nodes;
  std::vector<Graph::vertex> improvement_graph;

  for (auto insert_helper = path.begin();
       insert_helper != path.end(); insert_helper++)
  {
    improvement_nodes.push_back(
        Graph::Point{insert_helper->x, insert_helper->y, insert_helper->z});
  }

  for (auto i = improvement_nodes.begin(); i != --improvement_nodes.end();
       i++)
  {

    for (auto i2 = std::next(improvement_nodes.begin(), 1);
         i2 != improvement_nodes.end(); i2++)
    {

      ufo::math::Vector3 start(i->location.x, i->location.y,
                               i->location.z);
      ufo::math::Vector3 goal(i2->location.x, i2->location.y,
                              i2->location.z);

      double dist = calculateDistance(start, goal);

      ufo::geometry::OBB obb =
          makeOBB(start, goal, m_obb_radius);
      if (isInCollision(m_ufoMAP, obb, false, true, false,
                        m_planning_depth) &&
          !isInCollision(m_ufoMAP, obb, true, false, true,
                         m_planning_depth))
      {

        improvement_graph.push_back(*i);
        improvement_graph.push_back(*i2);
        (*i).createEdge(*i2);
        (*i2).createEdge(*i);
      }
    }
  }

  Graph::Point start_pos{improvement_graph.front().location.x, improvement_graph.front().location.y,
                     improvement_graph.front().location.z};

  Graph::Point end_pos{improvement_graph.back().location.x, improvement_graph.back().location.y,
                      improvement_graph.back().location.z};

  Graph improved_graph;

  std::vector<Graph::Point> improved_path = improved_graph.fetchPath(start_pos, end_pos, improvement_graph);

  ROS_INFO_STREAM ("Improved Path length is " << improved_path.size());

  nav_msgs::Path planned_path;
  geometry_msgs::PoseStamped p;

  for (int i = 0; i < improved_path.size(); ++i)
  {
   
    ufo::math::Vector3 prev (improved_path[i].x, improved_path[i].y, improved_path[i].z);
    ufo::math::Vector3 next (improved_path[i+1].x, improved_path[i+1].y, improved_path[i+1].z);
    ufo::math::Vector3 before_last (improved_path[i-1].x, improved_path[i-1].y, improved_path[i-1].z);
  
    float angle = atan2( next.y() - prev.y(), next.x() - prev.x());
    float angle_last = atan2( prev.y() - before_last.y(), prev.x() - before_last.x());
    ufo::math::Vector3 v(0, 0, angle);
    ufo::math::Vector3 v_last(0, 0, angle_last);
    Eigen::Quaternion<float> q = Euler2Quaternion(v);
    Eigen::Quaternion<float> q_last = Euler2Quaternion(v_last);

    if (i == improved_path.size()-1) {
      q = q_last;
    }
    p.pose.position.x = improved_path[i].x;
    p.pose.position.y = improved_path[i].y;
    p.pose.position.z = improved_path[i].z;
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    planned_path.poses.push_back(p);
 
  }

  if (planned_path.poses.size() < 2) {

    fetchExplorationPath();

  }

  ROS_INFO_STREAM ("Planned path length : " << planned_path.poses.size());
  
  // Final exploration path to be sent to stage interface for segmentation and execution.

  return planned_path;

} // fetchExplorationPath


void GraphManager::planHomingPath () {

  ufo::math::Vector3 home_pose_vec, source_vec;
  double d_s, d_g;
  double home_pose_distance, source_distance = std::numeric_limits<float>::max();

  for (Graph::vertex v : global_graph_container_)
  {

    ufo::math::Vector3 vertex_vector(v.location.x, v.location.y, v.location.z);

    double d_s = calculateDistance(vertex_vector, fetchCurrentPosition());
    double d_g = calculateDistance(vertex_vector, home_position);

    if (d_s < source_distance)
    {
      source_distance = d_s;
      source_vec = vertex_vector;
    }
    if (d_g < home_pose_distance)
    {
      home_pose_distance = d_g;
      home_pose_vec = vertex_vector;
    }
  }

  Graph::Point source{source_vec.x(), source_vec.y(), source_vec.z()};
  Graph::Point goal{home_pose_vec.x(), home_pose_vec.y(), home_pose_vec.z()};

  Graph homing_graph;
  std::vector<Graph::Point> path = homing_graph.fetchPath(source, goal, global_graph_container_);

 
  std::vector<Graph::vertex> improvement_nodes;
  std::vector<Graph::vertex> improvement_graph;

  for (auto insert_helper = path.begin();
       insert_helper != path.end(); insert_helper++)
  {
    improvement_nodes.push_back(
        Graph::Point{insert_helper->x, insert_helper->y, insert_helper->z});
  }

  for (auto i = improvement_nodes.begin(); i != --improvement_nodes.end();
       i++)
  {

    for (auto i2 = std::next(improvement_nodes.begin(), 1);
         i2 != improvement_nodes.end(); i2++)
    {

      ufo::math::Vector3 start(i->location.x, i->location.y,
                               i->location.z);
      ufo::math::Vector3 goal(i2->location.x, i2->location.y,
                              i2->location.z);

      double dist = calculateDistance(start, goal);

      ufo::geometry::OBB obb =
          makeOBB(start, goal, m_obb_radius);
      if (isInCollision(m_ufoMAP, obb, false, true, false,
                        m_planning_depth) &&
          !isInCollision(m_ufoMAP, obb, true, false, true,
                         m_planning_depth))
      {

        improvement_graph.push_back(*i);
        improvement_graph.push_back(*i2);
        (*i).createEdge(*i2);
        (*i2).createEdge(*i);
      }
    }
  }

  Graph::Point start{improvement_graph.front().location.x, improvement_graph.front().location.y,
                     improvement_graph.front().location.z};

  Graph::Point finish{improvement_graph.back().location.x, improvement_graph.back().location.y,
                      improvement_graph.back().location.z};

  Graph improved_graph;

  std::vector<Graph::Point> improved_path = improved_graph.fetchPath(start, finish, improvement_graph);

  nav_msgs::Path planned_path;
  geometry_msgs::PoseStamped p;

  for (int i = 0; i < improved_path.size(); ++i)
  {
   
    ufo::math::Vector3 prev (improved_path[i].x, improved_path[i].y, improved_path[i].z);
    ufo::math::Vector3 next (improved_path[i+1].x, improved_path[i+1].y, improved_path[i+1].z);
    ufo::math::Vector3 before_last (improved_path[i-1].x, improved_path[i-1].y, improved_path[i-1].z);
  
    float angle = atan2( next.y() - prev.y(), next.x() - prev.x());
    float angle_last = atan2( prev.y() - before_last.y(), prev.x() - before_last.x());
    ufo::math::Vector3 v(0, 0, angle);
    ufo::math::Vector3 v_last(0, 0, angle_last);
    Eigen::Quaternion<float> q = Euler2Quaternion(v);
    Eigen::Quaternion<float> q_last = Euler2Quaternion(v_last);

    if (i == improved_path.size()-1) {
      q = q_last;
    }
    p.pose.position.x = improved_path[i].x;
    p.pose.position.y = improved_path[i].y;
    p.pose.position.z = improved_path[i].z;
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    planned_path.poses.push_back(p);
 
  }


  if (planned_path.poses.size() > 1) {
    homing_path_ = planned_path;
  } else {
    ROS_WARN("Homing path not valid");
  }
 
}



bool GraphManager::nextExplorationPath(stage_planner::explorationPath::Request &req,
                                        stage_planner::explorationPath::Response &res)

{
  
  nav_msgs::Path exploration_path = GraphManager::fetchExplorationPath ();

  // TODO Homing path to be included in this ros service 

  if (req.exploration) {
    res.path = exploration_path;
    if (local_exploration) { 
      res.planning_status_local = true;
      res.planning_status_global = false;
    } else if (global_exploration ) {
      res.planning_status_global = true;
      res.planning_status_local = false;
    }
  } 

  if (req.homing) {
    res.path = homing_path_;
      res.planning_status_local = false;
      res.planning_status_global = false;
      res.planning_status_homing = true;
  }

  return true;

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
