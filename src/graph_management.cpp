#include "graph_management.h"
#include <iterator>
#include <utility>

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

GraphManager::GraphManager(ros::NodeHandle private_nh_)
    : m_nh(), m_worldFrameId("world"), m_odometry("spot/ground_truth"),
      m_currentHeading(0), m_local_space(10), m_local_fov(40),
      m_ufomap("/ufomap_mapping_server_node/map"), m_max_allowed_Z(4),
      m_obb_radius(0.5), m_planning_depth(0), m_volumetric_gain_threashold(10),
      m_min_dist_bw_nodes(0.8)

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
  private_nh.param("minimum_distance_between_nodes", m_min_dist_bw_nodes,
                   m_min_dist_bw_nodes);

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
};

ufo::map::OccupancyMapColor m_ufoMAP(0.2);

void GraphManager::mapCallback(
    ufomap_msgs::UFOMapStamped::ConstPtr const &msg) {

  if (ufomap_msgs::msgToUfo(msg->map, m_ufoMAP)) {
    // ROS_INFO("MAP RECEIVED");
    map_received = true;
  } else {
    ROS_WARN("MAP NOT RECEIVED");
  }
}

void GraphManager::updateCurrentPosition(
    const nav_msgs::Odometry::ConstPtr &pose) {

  if (firstTime) {
    m_prevPose = *pose;
    firstTime = false;
  } else {
    m_currentPose = *pose;
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

ufo::math::Vector3 GraphManager::fetchCurrentPosition() {
  return m_currentPosition;
}

double GraphManager::calculateDistance(ufo::math::Vector3 point1,
                                       ufo::math::Vector3 point2) {

  double d =
      sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) +
           pow(point1.z() - point2.z(), 2));
  return d;
}

bool GraphManager::isInCollision(
    ufo::map::OccupancyMapColor const &map,
    ufo::geometry::BoundingVar const &bounding_volume, bool occupied_space,
    bool free_space, bool unknown_space, ufo::map::DepthType min_depth) {

  for (auto it = map.beginLeaves(bounding_volume, occupied_space, free_space,
                                 unknown_space, false, min_depth),
            it_end = map.endLeaves();
       it != it_end; ++it) {
    return true;
  }
  return false;
}

ufo::geometry::OBB GraphManager::makeOBB(ufo::math::Vector3 source,
                                         ufo::math::Vector3 goal,
                                         float radius) {

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
                                    double heading) {

  double head =
      atan2((fpoint.y() - position.y()), (fpoint.x() - position.x())) *
      (180 / 3.14159);
  double angle = head - heading;
  if (head > -180 && head < -90) {
    if (heading > -180 && heading < -90) {
      angle = head - heading;
      if (angle > 180) {
        angle = angle - 360;
      }
    } else {
      angle = angle + 360;
      if (angle > 180) {
        angle = angle - 360;
      }
    }
  } else if (head > 0) {
    if (heading > -180 && heading < -90) {
      angle = angle - 360;
      if (angle > 180) {
        angle = angle - 360;
      }
    } else {
      angle = angle;
      if (angle > 180) {
        angle = angle - 360;
      }
    }
  }
  return angle;
}

bool GraphManager::isLocal(ufo::math::Vector3 point) {

  double angle =
      GraphManager::calculateAngle(point, m_currentPosition, m_currentHeading);
  double distance = GraphManager::calculateDistance(point, m_currentPosition);

  if ((angle < m_local_fov && angle > m_local_fov) && distance < m_local_space) {
    return true;
  } else {
    return false;
  }
}

double
GraphManager::computeVolumetricGain(ufo::math::Vector3 frontier,
                                    ufo::map::OccupancyMapColor const &map) {

  double r = 4;
  int unknown_cells, free_cells, occupied_cells = 0;
  ufo::geometry::Sphere sphere(frontier, r);

  for (auto it = map.beginLeaves(sphere, false, false, true, false,
                                 m_planning_depth),
            it_end = map.endLeaves();
       it != it_end; ++it) {

    unknown_cells++;
  }

  for (auto it = map.beginLeaves(sphere, false, true, false, false,
                                 m_planning_depth),
            it_end = map.endLeaves();
       it != it_end; ++it) {

    free_cells++;
  }

  for (auto it = map.beginLeaves(sphere, true, false, false, false,
                                 m_planning_depth),
            it_end = map.endLeaves();
       it != it_end; ++it) {

    occupied_cells++;
  }

  double volumetric_gain =
      10 * (log(unknown_cells + free_cells) / log(occupied_cells));

  return volumetric_gain;
}

ufo::math::Vector3
GraphManager::tunePlanningSpace(float range, ufo::math::Vector3 root_node,
                                ufo::map::OccupancyMapColor const &map) {

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
       it != map.endLeaves(); ++it) {
    if (it.getX() > highest_x) {
      highest_x = it.getX();
    }
    if (it.getX() < lowest_x) {
      lowest_x = it.getX();
    }
    if (it.getY() > highest_y) {
      highest_y = it.getY();
    }
    if (it.getY() < lowest_y) {
      lowest_y = it.getY();
    }
    if (it.getZ() > highest_z) {
      highest_z = it.getZ();
    }
    if (it.getZ() < lowest_z) {
      lowest_z = it.getZ();
    }
  }

  ufo::math::Vector3 planning_space_scaler(
      highest_x - lowest_x, highest_y - lowest_y, highest_z - lowest_z);
  return planning_space_scaler;
}

ufo::math::Vector3 *GraphManager::node::getNodePointer() { return point; }

void GraphManager::node::addChild(node *newChild) {
  myChilds.push_back(newChild);
}

void GraphManager::node::addParent(node *newParent) { myParent = newParent; }

void GraphManager::node::setTraversabilityStatus(bool Traversable) {
  isTraversableByDrone = Traversable;
}
bool GraphManager::node::getTraversabilityStatus() {
  return isTraversableByDrone;
}
void GraphManager::node::readyForDeletion() { delete point; }

std::vector<GraphManager::node *>
GraphManager::extractNodes(bool require_local, bool build_reserved_tree,
                           ufo::math::Vector3 root_node, ufo::math::Vector3 target, 
                           int max_iterations,
                           float range,
                           ufo::map::OccupancyMapColor const &map) {
  std::vector<node *> temporary_nodes;

  for (std::vector<node *>::iterator it_clear_helper = temporary_nodes.begin();
       it_clear_helper != --temporary_nodes.end(); it_clear_helper++) {
    (*it_clear_helper)->readyForDeletion();
    delete (*it_clear_helper);
  }

  temporary_nodes.clear();
  node *current_pos = new node(root_node.x(), root_node.y(), root_node.z());
  node *target_pos = new node(target.x(), target.y(), target.z());
  current_pos->addParent(nullptr);
  target_pos->addParent(nullptr);
  temporary_nodes.push_back(current_pos);
  temporary_nodes.push_back(target_pos);

  srand(time(0));
  int itterations = 0;

  while (itterations < max_iterations) {
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
                      m_planning_depth)) {
      float distance = std::numeric_limits<float>::max();
      node *parent;
      std::vector<node *>::iterator it_node;
      for (it_node = temporary_nodes.begin(); it_node != temporary_nodes.end();
           it_node++) {
        ufo::math::Vector3 direction = random_point - *((*it_node)->point);
        double new_distance = abs(direction.norm());
        if (new_distance < distance) {
          distance = new_distance;
          parent = *it_node;
        }
      };
      ufo::math::Vector3 start_point(parent->point->x(), parent->point->y(),
                                     parent->point->z());
      ufo::math::Vector3 direction = random_point - start_point;
      ufo::geometry::LineSegment myline(random_point, start_point);
      // ufo::geometry::OBB obb = makeOBB (start_point, random_point,
      // RAW_GRAPH_COLISSION_MARGIN);
      double min_dist = m_min_dist_bw_nodes / 2;
      if (require_local) {
        min_dist = min_dist;
      } else {
        min_dist = m_min_dist_bw_nodes;
      }
      if (!isInCollision(map, myline, true, false, true,
                         m_planning_depth) and
          calculateDistance(random_point, start_point) > min_dist)

      {
        node *new_node = new node(x, y, z);
        new_node->addParent(parent);
        parent->addChild(new_node);
        temporary_nodes.push_back(new_node);
      }
    };
    itterations++;
  };

  if (!build_reserved_tree) {
    ROS_INFO_STREAM("Planning space contains --> " << temporary_nodes.size()
                                                   << " nodes\n");
    return temporary_nodes;
  } else {
    for (auto it = temporary_nodes.begin(); it != temporary_nodes.end(); it++) {
      reserved_nodes.push_back(*it);
    }
    return reserved_nodes;
  }
}

std::vector<GraphManager::Graph::Point>
GraphManager::Graph::fetchPath(Point &start_pos, Point &end_pos,
                               std::vector<vertex> all_vertices) {
  vertex start_vert;
  vertex end_vert;

  for (vertex &vert : all_vertices) {
    if (vert.location == end_pos)
      start_vert = vert;
    if (vert.location == start_pos)
      end_vert = vert;
  }

  std::vector<Point> found_path; // Output path
  std::set<vertex> open_list;
  std::vector<Point> closed_list;

  open_list.emplace(start_vert);

  while (!open_list.empty()) {
    vertex current_vert = *open_list.begin();

    open_list.erase(open_list.begin());
    closed_list.push_back(current_vert.location);

    if (current_vert.location == end_vert.location) {
      vertex *parent = &current_vert;

      while (parent != nullptr) {
        found_path.push_back(parent->location);
        parent = parent->parent.get();
      }
      return (found_path);
    }

    for (vertex &vert : *current_vert.edges) {
      vertex child(vert);
      child.parent.reset(new vertex(current_vert));

      if (std::find_if(closed_list.begin(), closed_list.end(),
                       [&child](Point &pt) {
                         return (pt == child.location);
                       }) != closed_list.end())
        continue;

      child.dist_to_start = current_vert.dist_to_start +
                            child.location.DistTo(current_vert.location);

      child.total_dist = 0.1 * (child.dist_to_start +
                                (child.location.DistTo(end_vert.location)));

      if (std::find_if(open_list.begin(), open_list.end(),
                       [&child](const vertex &vert) {
                         return ((vert == child) &&
                                 (child.dist_to_start > vert.dist_to_start));
                       }) != open_list.end())
        continue;

      open_list.emplace(child);
    }
  }

  return (found_path);
}

ufo::math::Vector3 GraphManager::pointToUfo(geometry_msgs::Point p) {
  ufo::math::Vector3 v(p.x, p.y, p.z);
  return v;
}

std::vector<ufo::math::Vector3>
GraphManager::pointsToUfo(std::vector<geometry_msgs::Point> points) {

  std::vector<ufo::math::Vector3> ufo_vec;

  for (geometry_msgs::Point p : points) {
    ufo_vec.push_back(ufo::math::Vector3(p.x, p.y, p.z));
  }

  return ufo_vec;
}

void GraphManager::fetchExplorationTargets() {

  stage_planner::frontierRequest frontier_srv;

  frontier_srv.request.local = true;

  if (m_frontier_client.call(frontier_srv)) {

    if (frontier_srv.response.local_nav) {

      m_local_exploration_target = pointToUfo(frontier_srv.response.best_local);

    } else if (frontier_srv.response.local_nav == false) {

      m_global_repos_targets =
          pointsToUfo(frontier_srv.response.global_targets);
    }
  }
}

void GraphManager::constructLocalGraph() {

  fetchExplorationTargets();

  std::vector<GraphManager::node *> nodes =
      extractNodes(true, false, m_currentPosition, m_local_exploration_target,
                   m_max_iterations_local, m_local_space, m_ufoMAP);

  std::vector<std::pair<ufo::math::Vector3, ufo::math::Vector3>> local_nodes_pairs;

  for (auto i = nodes.begin(); i != std::prev(nodes.end(),1); i++) {
        
    for (auto i2 = i; i2 != nodes.end(); i2++) {

      ufo::math::Vector3 v1 ((*i)->myParent->point->x(), (*i)->myParent->point->y(), (*i)->myParent->point->z());
      ufo::math::Vector3 v2 ((*i2)->myParent->point->x(), (*i2)->myParent->point->y(), (*i2)->myParent->point->z());
      
      double distance = calculateDistance(v1, v2);
      
      if (distance < m_max_edge_length) {
        local_nodes_pairs.push_back(std::make_pair(v1,v2));
      }
    }
  }
}
