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
      m_local_space(12), 
      m_local_fov(40),
      m_ufomap("/ufomap_mapping_server_node/map"), 
      m_max_allowed_Z(4),
      m_obb_radius(0.5), 
      m_planning_depth(1), 
      m_volumetric_gain_threashold(1),
      m_min_dist_bw_nodes(0.3), 
      m_iterations_local(2000), 
      m_iterations_global(6000),
      m_max_edge_length(4)

{

  // Set variables
  nh_private_.param("frame_id", m_worldFrameId, m_worldFrameId);
  nh_private_.param("odometry", m_odometry, m_odometry);

  nh_private_.param("local_sliding_space", m_local_space, m_local_space);
  nh_private_.param("local_field_of_view", m_local_fov, m_local_space);
  nh_private_.param("ufomap", m_ufomap, m_ufomap);
  nh_private_.param("safety_height", m_max_allowed_Z, m_max_allowed_Z);
  nh_private_.param("traversability_radius", m_obb_radius, m_obb_radius);
  nh_private_.param("planning_check_depth", m_planning_depth,
                   m_planning_depth);
  nh_private_.param("volumetric_gain_threashold", m_volumetric_gain_threashold,
                   m_volumetric_gain_threashold);
  nh_private_.param("graph_density_coefficient", m_min_dist_bw_nodes,
                   m_min_dist_bw_nodes);
  nh_private_.param("maximum_edge_length", m_max_edge_length, m_max_edge_length);
  nh_private_.param("maximum_iterations_local", m_iterations_local, m_iterations_local);
  nh_private_.param("maximum_iterations_global", m_iterations_global, m_iterations_global);

  // Subscribers

    m_poseSubscriber = nh_.subscribe(m_odometry, 1, &GraphManager::updateCurrentPosition, this);

    m_ufoMapSubscriber = nh_.subscribe(m_ufomap, 1, &GraphManager::mapCallback, this);

    // publishers  

    m_frontierPub = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_nodes_vis", 10);


  // Services 
  m_frontier_client = nh_.serviceClient<stage_planner::frontierRequest>("request_frontier_target");
  
  m_target_query_client = nh_.serviceClient<stage_planner::isFrontier>("query_frontier_target");

  
  m_exploration_path_srv = nh_.advertiseService("next_exploration_path", &GraphManager::nextExplorationPath, this);

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

bool GraphManager::nextExplorationPath(stage_planner::explorationPath::Request &req,
                                        stage_planner::explorationPath::Response &res)

{


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

/*

TODO:
  
  1. Homing planner to be running in the background -- done 

  2. Rosservice to fetch new exploration path

  3. Convert path to trajectory in stage_interface.

  4. Add initialization behaviour in stage_interface.

  3. Rosservice to fetch return to home path 

  4. Rosnode health monitoring 

*/
