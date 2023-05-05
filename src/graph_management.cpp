#include "graph_management.h"

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

GraphManager::GraphManager(ros::NodeHandle private_nh_)
    : m_nh(),
    m_worldFrameId("world"),
    m_odometry("spot/ground_truth"),
    m_currentHeading(0),
    LOCAL_SPACE(10),
    LOCAL_FOV(40),    
    m_ufomap("/ufomap_mapping_server_node/map"),
    m_max_allowed_Z(4),  
    OBB_RADIUS(0.5),
    PLANNING_CHECK_DEPTH(0),
    VOLUMETRIC_GAIN_THREASHOLD(10)




{


   

    ros::NodeHandle private_nh = private_nh_;
    //Set variables
    private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
    private_nh.param("odometry", m_odometry, m_odometry);
    
    private_nh.param("local_sliding_space", LOCAL_SPACE, LOCAL_SPACE);
    private_nh.param("local_field_of_view", LOCAL_FOV, LOCAL_SPACE);
    private_nh.param("ufomap", m_ufomap, m_ufomap);
    private_nh.param("safety_height", m_max_allowed_Z, m_max_allowed_Z);
    private_nh.param("traversability_radius", OBB_RADIUS, OBB_RADIUS);
    private_nh.param("planning_check_depth", PLANNING_CHECK_DEPTH, PLANNING_CHECK_DEPTH);
    private_nh.param("volumetric_gain_threashold", VOLUMETRIC_GAIN_THREASHOLD, VOLUMETRIC_GAIN_THREASHOLD);






    //Register to the subscriber topic and register it to the callback function

    m_poseSubscriber = new message_filters::Subscriber<nav_msgs::Odometry> (m_nh, m_odometry, 5);
    m_poseSubscriber->registerCallback(boost::bind(&GraphManager::updateCurrentPosition, this, _1));

    m_ufoMapSubscriber = new message_filters::Subscriber<ufomap_msgs::UFOMapStamped> (m_nh, m_ufomap, 1);
    m_ufoMapSubscriber->registerCallback(boost::bind(&GraphManager::mapCallback, this, _1));

    //Register publishers
    m_frontierPub = m_nh.advertise<visualization_msgs::MarkerArray>("/frontier_nodes_vis", 1, true);



};

ufo::map::OccupancyMapColor m_ufoMAP(0.2);

void GraphManager::mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg)
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



void GraphManager::updateCurrentPosition(const nav_msgs::Odometry::ConstPtr& pose) {

    if (firstTime) {
        m_prevPose = *pose;
        firstTime = false;
    } else {
        m_currentPose = *pose;
    }
    m_currentPosition.x() = m_currentPose.pose.pose.position.x;
    m_currentPosition.y() = m_currentPose.pose.pose.position.y;
    m_currentPosition.z() = m_currentPose.pose.pose.position.z;

    tf::Quaternion q(
      pose->pose.pose.orientation.x,
      pose->pose.pose.orientation.y,
      pose->pose.pose.orientation.z,
      pose->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll, pitch, yaw);
    m_currentHeading = yaw * (180 / 3.14159);

}

ufo::math::Vector3 GraphManager::fetchCurrentPosition () {
  return m_currentPosition;
}


double GraphManager::calculateDistance (ufo::math::Vector3 point1, ufo::math::Vector3 point2) {

    double d = sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) + pow(point1.z() - point2.z(), 2));
    return d;

}

bool GraphManager::isInCollision(ufo::map::OccupancyMapColor const &map,
                   ufo::geometry::BoundingVar const &bounding_volume,
                   bool occupied_space, bool free_space,
                   bool unknown_space, ufo::map::DepthType min_depth) {

  for (auto it = map.beginLeaves(bounding_volume, occupied_space,
                                 free_space, unknown_space, false, min_depth),
            it_end = map.endLeaves();
       it != it_end; ++it)
  {
    return true;
  }
  return false;
}

ufo::geometry::OBB GraphManager::makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius)
{

  ufo::math::Vector3 direction = goal - source;
  ufo::math::Vector3 center = source + (direction / 2.0);
  double distance = direction.norm();
  direction /= distance;
  double yaw = -atan2(direction[1], direction[0]);
  double pitch = -asin(direction[2]);
  double roll = 0;
  ufo::geometry::OBB obb(center, ufo::map::Point3(distance / 2.0, radius, radius),
                         ufo::math::Quaternion(roll, pitch, yaw));

  return obb;
}

double GraphManager::calculateAngle(ufo::math::Vector3 fpoint, ufo::math::Vector3 position,
                      double heading)
{

  double head = atan2((fpoint.y() - position.y()), (fpoint.x() - position.x())) * (180 / 3.14159);
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

bool GraphManager::isLocal (ufo::math::Vector3 point) {

    double angle = GraphManager::calculateAngle (point, m_currentPosition, m_currentHeading);
    double distance = GraphManager::calculateDistance (point, m_currentPosition);

    if ((angle < LOCAL_FOV && angle > LOCAL_FOV) && distance < LOCAL_SPACE) {
        return true;
    } else { return false; }

}



double GraphManager::computeVolumetricGain (ufo::math::Vector3 frontier, ufo::map::OccupancyMapColor const &map) {
    
    double r = 4;
    int unknown_cells, free_cells, occupied_cells = 0;
    ufo::geometry::Sphere sphere (frontier, r);

    for (auto it = map.beginLeaves(sphere, false, false, true, false, PLANNING_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it)  {
        
        unknown_cells++;
    }

    for (auto it = map.beginLeaves(sphere, false, true, false, false, PLANNING_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it)  {
        
        free_cells++;
    }

    for (auto it = map.beginLeaves(sphere, true, false, false, false, PLANNING_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it)  {
        
        occupied_cells++;
    }

    double volumetric_gain = 10 * (log(unknown_cells + free_cells ) / log (occupied_cells));

    return volumetric_gain;
}

    

