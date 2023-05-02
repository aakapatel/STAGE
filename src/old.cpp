#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_msgs/UFOMapMetaData.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_ros/conversions.h>
#include <ufo/map/types.h>
#include <ufo/math/vector3.h>
#include <ros/ros.h>
#include <list>
#include <iostream>
#include <chrono>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <tf/tf.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <acl_exploration/ServerConfig.h>
#include "acl_exploration/Frontiers.h"
#include "acl_exploration/path.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <limits>
#include <set>
#include <utility>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <climits>
#include <memory>
#include <fstream>
#include <variant>
#include <shared_mutex>
using namespace std::chrono;
using namespace std;
using namespace std::this_thread;
double PLANNING_CHECK_DEPTH;
struct node
{
public:
  ufo::math::Vector3 *point;
  node *myParent = nullptr;
  double distanceToParent = -1;
  std::list<struct node *> myChilds{};
  std::list<struct node *> myPath{};
  
  double distanceToGoal;
  node(float x, float y, float z)
  {
    point = new ufo::math::Vector3(x, y, z);
  }
  node(ufo::math::Vector3 *givenPoint)
  {
    point = new ufo::math::Vector3(givenPoint->x(), givenPoint->y(), givenPoint->z());
  }
  ufo::math::Vector3 *getNodePointer()
  {
    return point;
  }
  void addChild(node *newChild)
  {
    myChilds.push_back(newChild);
  }
  void addParent(node *newParent)
  {
    myParent = newParent;
  }
  void changeDistanceToGoal(double newDistance)
  {
    distanceToGoal = newDistance;
  }
  void changeDistanceToParent()
  {
    if (myParent != nullptr and distanceToParent != -1)
    {
      distanceToParent = sqrt(pow(point->x() - myParent->point->x(), 2) + pow(point->y() - myParent->point->y(), 2) + pow(point->z() - myParent->point->z(), 2));
    }
    else
    {
      distanceToParent = 0;
    }
  }
  double sumDistance()
  {
    changeDistanceToParent();
    double myDistance = 0;
    if (myParent != nullptr)
    {
      myDistance = myParent->sumDistance();
      changeDistanceToParent();
    }
    if (myParent == nullptr)
    {
      return 0;
    }
    return (myDistance + distanceToParent);
  }

  void getPath(std::list<struct node *> *givenPath)
  {
    if (myParent != nullptr)
    {
      myParent->getPath(givenPath);
      if (myParent->myParent != nullptr)
      {
        if (myParent->point->x() != point->x() or myParent->point->y() != point->y() or myParent->point->z() != point->z())
        {
          givenPath->push_back(new node(myParent->point->x(), myParent->point->y(), myParent->point->z()));
        }
      }
    }
    if (myParent == nullptr)
    {
      return;
    }
    return;
  }

  bool findPathImprovement(struct node *targetNode, ufo::map::OccupancyMapColor const &map, float givenDistance, float givenRadious, auto pathImprovement_start, int givenMax)
  {
    bool improvementFound;
    if (targetNode == this and myParent == nullptr)
    {
      return true;
    }
    if (myParent != nullptr)
    {
      improvementFound = myParent->findPathImprovement(targetNode, map, givenDistance, givenRadious, pathImprovement_start, givenMax);
      auto pathImprovement_stop = high_resolution_clock::now();
      auto pathImprovement_total = duration_cast<microseconds>(pathImprovement_stop - pathImprovement_start).count();
      if (pathImprovement_total > givenMax)
      {
        return true;
      }
    }
    else
    {
      ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
      if (!isInCollision(map, myLine, true, false, true, PLANNING_CHECK_DEPTH))
      {
        ufo::math::Vector3 newVector(targetNode->point->x() - point->x(), targetNode->point->y() - point->y(), targetNode->point->z() - point->z());
        float distance = newVector.norm();
        // float distance = sqrt(pow(targetNode->point->x() - point->x(), 2) + pow(targetNode->point->y() - point->y(), 2) + pow(targetNode->point->z() - point->z(), 2));
        float itterations = (distance / givenRadious);
        float part = givenRadious / distance;
        float xStep = (targetNode->point->x() - point->x()) * part;
        float yStep = (targetNode->point->y() - point->y()) * part;
        float zStep = (targetNode->point->z() - point->z()) * part;
        for (int i = 1; i < itterations; i++)
        {
          auto pathImprovement_stop = high_resolution_clock::now();
          auto pathImprovement_total = duration_cast<microseconds>(pathImprovement_stop - pathImprovement_start).count();
          if (pathImprovement_total > givenMax)
          {
            return true;
          }
          ufo::math::Vector3 newVector = ufo::math::Vector3(point->x() + i * xStep, point->y() + i * yStep, point->z() + i * zStep);
          ufo::geometry::Sphere new_sphere(newVector, givenRadious);
          if (isInCollision(map, new_sphere, true, false, true, PLANNING_CHECK_DEPTH))
          {
            return false;
          }
        }
        targetNode->addParent(this);
        return true;
      }
      else
      {
        return false;
      };
    }
    if (!improvementFound)
    {
      ufo::geometry::LineSegment myLine(*(targetNode->point), *point);
      if (!isInCollision(map, myLine, true, false, true, PLANNING_CHECK_DEPTH))
      {
        ufo::math::Vector3 newVector(targetNode->point->x() - point->x(), targetNode->point->y() - point->y(), targetNode->point->z() - point->z());
        float distance = newVector.norm();
        float itterations = (distance / givenRadious);
        float part = givenRadious / distance;
        float xStep = (targetNode->point->x() - point->x()) * part;
        float yStep = (targetNode->point->y() - point->y()) * part;
        float zStep = (targetNode->point->z() - point->z()) * part;
        for (int i = 1; i < itterations; i++)
        {
          auto pathImprovement_stop = high_resolution_clock::now();
          auto pathImprovement_total = duration_cast<microseconds>(pathImprovement_stop - pathImprovement_start).count();
          if (pathImprovement_total > givenMax)
          {
            return true;
          }
          ufo::math::Vector3 newVector = ufo::math::Vector3(point->x() + i * xStep, point->y() + i * yStep, point->z() + i * zStep);
          ufo::geometry::Sphere new_sphere(newVector, givenRadious);
          if (isInCollision(map, new_sphere, true, false, true, PLANNING_CHECK_DEPTH))
          {
            return false;
          }
        }
        targetNode->addParent(this);
        improvementFound = findPathImprovement(this, map, givenDistance, givenRadious, pathImprovement_start, givenMax);
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return true;
    }
  }

  void readyForDeletion()
  {
    delete point;
  }

  bool isInCollision(ufo::map::OccupancyMapColor const &map,
                     ufo::geometry::BoundingVar const &bounding_volume,
                     bool occupied_space = true, bool free_space = false,
                     bool unknown_space = false, ufo::map::DepthType min_depth = PLANNING_CHECK_DEPTH)
  {
    // Iterate through all leaf nodes that intersects the bounding volume
    for (auto it = map.beginLeaves(bounding_volume, occupied_space,
                                   free_space, unknown_space, false, min_depth),
              it_end = map.endLeaves();
         it != it_end; ++it)
    {
      // Is in collision since a leaf node intersects the bounding volume.
      return true;
    }
    // No leaf node intersects the bounding volume.
    return false;
  }
};




class Graph
{

public:
  struct Point
  {
    Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z){};
    bool operator==(const Point &o) const { return o.x == x && o.y == y && o.z == z; }
    double DistTo(Point &other) const { return (std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2) + std::pow(other.z - z, 2))); };
    double Dist_HeuristicTo(Point &other) const { return (0.1 * (std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2) + std::pow(other.z - z, 2)))); };
    // double DistToHeuristic(Point & other) const { return(abs(other.x - x) + abs(other.y - y) + abs(other.z - z)); };
    double x, y, z;
  };
  struct planning_node
  {

    planning_node(Point myposition = Point(), planning_node *parent = nullptr) : myposition(myposition), parent(parent){};
    // planning_node(Point myposition = {}, std::shared_ptr<planning_node> parent = {} ): myposition(myposition), parent(parent) {};
    Point myposition;

    double dist_to_start = 0;
    double total_dist = INT_MAX;

    std::shared_ptr<planning_node> parent;
    std::shared_ptr<std::vector<planning_node>> connections{new std::vector<planning_node>};
    // std::vector< std::shared_ptr<planning_node> > connections{ new std::vector<planning_node> };
    bool operator==(const planning_node &o) const { return (o.myposition.x == myposition.x) && (o.myposition.y == myposition.y) && (o.myposition.z == myposition.z); }
    bool operator<(const planning_node &o) const { return (total_dist < o.total_dist); }

    void AddConnection(planning_node subnode)
    {
      connections->push_back(subnode);
    }
    bool connection_exist(){
        if (connections->size() < 5) {
            return true;
        } else {
            return false;
        }
    }
  };

  std::vector<Point> fetchPath(Point &start_pos, Point &end_pos, std::vector<planning_node> all_nodes, ufo::map::OccupancyMapColor const &map);
};



struct Frontier_main
{
  acl_exploration::path path;
  bool navigate;
  bool improve;
  std::list<ufo::math::Vector3> list;
  ufo::math::Vector3 position;
  geometry_msgs::PoseStamped pose;
  double heading;
  double x_ref;
  double y_ref;
  double z_ref;
  std::vector<Graph::planning_node> node_container;
  node *graph_pointer;
  std::vector<Graph::Point> traj;
  std::vector<ufo::math::Vector3> trajectory;
  ros::Publisher points_pub;
  ros::Publisher robot_box_vis_pub;
  ros::Publisher current_goal_vis_pub, current_unknowns_visualization_pub;
  ros::Publisher alc_points_visualization_pub;
  ros::Publisher alc_path_visualization_pub;
  ros::Publisher map_pub;
  ros::Publisher hits_pub;
  ros::Publisher local_nodes_pub;
  ros::Publisher global_nodes_pub;
  ros::Publisher explored_path;
  ros::Publisher path_to_goal_pub;
  ros::Publisher local_path_pub;
  ros::Publisher global_path_pub;
  ros::Publisher homing_path_pub;
  ros::Publisher best_frontier_pub;
  geometry_msgs::PoseStamped reference;
  ros::Publisher total_execution_pub;
  ros::Publisher local_execution_pub;
  ros::Publisher global_execution_pub;
  nav_msgs::Path temppath_homing;
  nav_msgs::Path temppath_local;
  nav_msgs::Path temppath_global;
  ufo::math::Vector3 end_pt;
  std::list<ufo::math::Vector3> unknowns;
};

bool position_received;
bool map_received = false;
int itterations;
float SCALER_X;
float SCALER_Y;
float SCALER_Z;
float position_x = 0;
float position_y = 0;
float position_z = 0;
float velocity_x = 0;
float velocity_y = 0;
float velocity_z = 0;
float lowest_x;
float lowest_y;
float lowest_z;
float highest_x;
float highest_y;
float highest_z;
double NEXT_PUB_DISTANCE;
double LOCAL_RANGE;
double RADIOUS;
double RESOLUTION;
double roll = 0;
double pitch = 0;
double yaw = 0;
double totalCost = std::numeric_limits<float>::max();
double totalDistance = -1;
node *goalNode = nullptr;
node *reserveGoalNode = nullptr;
node *currentTarget;
double md;
double head_ref;
bool firstTime = true;
bool start_flag = true;
bool move_flag = false;
bool travers;
bool frontiers_received = false;
bool INITIALIZE;
double FIRST_REF_X;
double FIRST_REF_Y;
double FIRST_REF_Z;
double MIN_DIST_NODE;
double RAW_GRAPH_COLISSION_MARGIN;
double PLANNING_GRAPH_COLISSION_MARGIN;
double RAW_FRONTIERS_COLISSION_MARGIN;

double MIN_ITERATION_LOCAL;
double MIN_ITERATION_GLOBAL;
double MIN_PLANNING_SPACE_LOCAL;
double MIN_PLANNING_SPACE_GLOBAL;
double MAX_ITERATION_LOCAL;
double MAX_ITERATION_GLOBAL;
double MAX_PLANNING_SPACE_LOCAL;
double MAX_PLANNING_SPACE_GLOBAL;
double IMPROVEMENT_SCALE_FACTOR_IT;
double IMPROVEMENT_SCALE_FACTOR_RANGE;

std::string ODOMETRY_IN;
std::string MAP_TOPIC;
std::string ODOM_FRAME_ID;
std::string REF_TOPIC;
std::string PATH_TOPIC;
nav_msgs::Odometry m_currentPose;
nav_msgs::Odometry m_prevPose;
std::list<ufo::math::Vector3> ext;
std::list<ufo::math::Vector3> accessible;
std::list<ufo::math::Vector3> positionslist;

std::list<struct node *> Global_Graph{};
std::list<struct node *> Local_Planning_Graph{};
std::list<struct node *> skeleton{};
std::list<struct node *> CHOSEN_PATH{};
std::list<double> CHOSEN_PATH_VREF{};
std::list<double>::iterator vref_itterator;
std::list<struct node *> ALL_PATH{};
std::list<struct node *> myGoals{};
std::list<struct node *> myReserveGoals{};
std::list<ufo::math::Vector3> hits{};
std::list<node *> VISITED_POINTS{};
std::list<node *> ALC_POINTS{};
std::list<node *>::iterator path_itterator;

Frontier_main all;
Frontier_main local;
Frontier_main global;
Frontier_main current;
Frontier_main old_poses;
Frontier_main traversable;
Frontier_main pub;
Frontier_main max_information;
Frontier_main global_frontier;
Frontier_main planning;
Frontier_main planned;
Frontier_main improved;
Frontier_main published;
Frontier_main starting;
Frontier_main ending;
Frontier_main home;
Frontier_main explore;
Frontier_main switch_global;
Frontier_main temp_local;
Frontier_main finish;
Frontier_main start_backtrack;
Frontier_main resume;
Frontier_main init;
Frontier_main lookAt;
Frontier_main final;

ufo::map::OccupancyMapColor ufoMAP(RESOLUTION);

// ufo::map::OccupancyMapSmall MAP(RESOLUTION);

// home.navigate = false;

//--------------------------------- FUNCTIONS ----------------------------------------------//

double calculateDistance(ufo::math::Vector3 point1, ufo::math::Vector3 point2)
{
  double dist = sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) + pow(point1.z() - point2.z(), 2));
  return dist;
}

bool isInCollision(ufo::map::OccupancyMapColor const &map,
                   ufo::geometry::BoundingVar const &bounding_volume,
                   bool occupied_space = true, bool free_space = false,
                   bool unknown_space = false, ufo::map::DepthType min_depth = PLANNING_CHECK_DEPTH)
{
  for (auto it = map.beginLeaves(bounding_volume, occupied_space,
                                 free_space, unknown_space, false, min_depth),
            it_end = map.endLeaves();
       it != it_end; ++it)
  {
    return true;
  }
  return false;
}

ufo::geometry::OBB makeOBB (ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius) {

  ufo::math::Vector3 direction = goal - source;
  ufo::math::Vector3 center = source + (direction / 2.0);
  double distance = direction.norm();
  direction /= distance;
  double yaw = -atan2(direction[1], direction[0]);
  double pitch = -asin(direction[2]);
  double roll = 0; // TODO: Fix
  ufo::geometry::OBB obb(center, ufo::map::Point3(distance / 2.0, radius, radius),
                          ufo::math::Quaternion(roll, pitch, yaw));
  
  return obb;

}

// TUNE GRAPH GENERATION. LOCAL AND GLOBAL PLANNING SPACE CONFIGURED HERE.

void planningSpace(bool local_graph, ufo::math::Vector3 end_point, float range, ufo::map::OccupancyMapColor const &map, bool occupied_space, bool free_space, bool unknown_space, float given_x,
                   float given_y, float given_z, ufo::map::DepthType min_depth = PLANNING_CHECK_DEPTH)
{

  highest_x = -9999;
  highest_y = -9999;
  highest_z = -9999;
  lowest_x = std::numeric_limits<float>::max();
  lowest_y = std::numeric_limits<float>::max();
  lowest_z = std::numeric_limits<float>::max();

    ufo::math::Vector3 minPoint(given_x - 1 * range, given_y - 1 * range, given_z - 1 * range);
    ufo::math::Vector3 maxPoint(given_x + 1 * range, given_y + 1 * range, given_z + 1 * range);
    ufo::geometry::AABB aabb(minPoint, maxPoint);
    ufo::math::Vector3 current_point(given_x, given_y, given_z);
    ufo::geometry::OBB obb = makeOBB(current_point, end_point, 10);

    ufo::geometry::BoundingVar PLANNING_GEOMETRY;

    if (local_graph) {
      PLANNING_GEOMETRY = aabb;
    } else {
      PLANNING_GEOMETRY = aabb;
    }

    for (auto it = map.beginLeaves(PLANNING_GEOMETRY, occupied_space, free_space, unknown_space, false, min_depth), it_end = map.endLeaves(); it != it_end; ++it)
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
  

  SCALER_X = highest_x - lowest_x;
  SCALER_Y = highest_y - lowest_y;
  SCALER_Z = highest_z - lowest_z;
}

void buildGraph(float given_x, float given_y, float given_z, ufo::math::Vector3 final_point, int max_iterations)
{
  for (std::list<node *>::iterator it_clear_helper = Global_Graph.begin(); it_clear_helper != --Global_Graph.end(); it_clear_helper++)
  {
    (*it_clear_helper)->readyForDeletion();
    delete (*it_clear_helper);
  }

  Global_Graph.clear();
  // std::cout << "\nBuilding graph... " << std::endl;
  node *current_pos = new node(given_x, given_y, given_z);
  // node *goal_pt = new node(final_point.x(), final_point.y(), final_point.z());
  // goal_pt->addParent(nullptr);
  current_pos->addParent(nullptr);
  Global_Graph.push_back(current_pos);
  // Global_Graph.push_back(goal_pt);

  srand(time(0));
  itterations = 0;
  while (itterations < max_iterations)
  {
    // Generate a random point
    float x = lowest_x + abs(1024 * rand() / (RAND_MAX + 1.0)) * SCALER_X;
    float y = lowest_y + abs(1024 * rand() / (RAND_MAX + 1.0)) * SCALER_Y;
    float z = lowest_z + abs(1024 * rand() / (RAND_MAX + 1.0)) * SCALER_Z;
    // float z = current.position.z();
    ufo::math::Vector3 random_point(x, y, z);
    ufo::geometry::Sphere point_sphere(random_point, RADIOUS);
    if (!isInCollision(ufoMAP, point_sphere, true, false, true, PLANNING_CHECK_DEPTH) and isInCollision(ufoMAP, point_sphere, false, true, false, PLANNING_CHECK_DEPTH))
    {
      float distance = std::numeric_limits<float>::max();
      node *parent;
      std::list<node *>::iterator it_node;
      for (it_node = Global_Graph.begin(); it_node != Global_Graph.end(); it_node++)
      {
        ufo::math::Vector3 direction = random_point - *((*it_node)->point);
        double new_distance = abs(direction.norm());
        if (new_distance < distance)
        {
          distance = new_distance;
          parent = *it_node;
        }
      };
      ufo::math::Vector3 start_point(parent->point->x(), parent->point->y(), parent->point->z());
      ufo::math::Vector3 direction = random_point - start_point;
      ufo::geometry::LineSegment myline(random_point, start_point);
      // ufo::geometry::OBB obb = makeOBB (start_point, random_point, RAW_GRAPH_COLISSION_MARGIN);
      if (!isInCollision(ufoMAP, myline, true, false, true, PLANNING_CHECK_DEPTH) 
                and calculateDistance(random_point, start_point) > MIN_DIST_NODE)
        // if (isCollisionFreeLine(ufoMAP, start_point, random_point, 0))

        {
          node *new_node = new node(x, y, z);
          new_node->addParent(parent);
          parent->addChild(new_node);
          Global_Graph.push_back(new_node);
      }
    };
    itterations++;
  };
  std::cout << "\nGraph built successfully with " << Global_Graph.size() << " nodes\n"
            << std::endl;
};

std::vector<Graph::Point> Graph::fetchPath(Point &start_pos, Point &end_pos, std::vector<planning_node> all_nodes, ufo::map::OccupancyMapColor const &map)
{

  planning_node start_node;
  planning_node end_node;

  for (planning_node &node : all_nodes)
  {
    if (node.myposition == end_pos)
      start_node = node;
    if (node.myposition == start_pos)
      end_node = node;
  }

  std::vector<Point> found_path; // Output path
  std::set<planning_node> open_list;
  std::vector<Point> closed_list;

  open_list.emplace(start_node);

  while (!open_list.empty())
  {
    planning_node current_node = *open_list.begin();

    open_list.erase(open_list.begin());
    closed_list.push_back(current_node.myposition);

    if (current_node.myposition == end_node.myposition)
    {
      planning_node *parent = &current_node;

      while (parent != nullptr)
      {
        found_path.push_back(parent->myposition);
        parent = parent->parent.get();
      }
      return (found_path);
    }

    // cout << "found path size before .. " << found_path.size() << endl;

    for (planning_node &node : *current_node.connections)
    {
      planning_node child(node);
      child.parent.reset(new planning_node(current_node));

      if (std::find_if(closed_list.begin(), closed_list.end(), [&child](Point &pt)
                       { return (pt == child.myposition); }) != closed_list.end())
        continue;

      child.dist_to_start = current_node.dist_to_start + child.myposition.DistTo(current_node.myposition);

      child.total_dist = child.dist_to_start + 0.05 * (child.myposition.DistTo(end_node.myposition));

      if (std::find_if(open_list.begin(), open_list.end(), [&child](const planning_node &node)
                       { return ((node == child) && (child.dist_to_start > node.dist_to_start)); }) != open_list.end())
        continue;

      open_list.emplace(child);
    }
  }

  return (found_path);
}


void visualize()
{
  visualization_msgs::Marker GRAPH_nodes, GRAPH_line_list, CURRENT_GOAL, CHOSEN_PATH_line_list,
      ALC_points, current_unknowns, current_unknown_line_list, local_unknowns, global_unknowns, ALC_line_list, POSITION_point, TRAVERSABLE_points,
      local_frontier_nodes, global_frontier_nodes;

  if (!current.unknowns.empty())
  {
    current_unknowns.header.frame_id = current_unknown_line_list.header.frame_id = ODOM_FRAME_ID;
    current_unknowns.ns = "all";
    current_unknowns.action = visualization_msgs::Marker::ADD;
    current_unknowns.pose.orientation.w = 1.0;
    current_unknowns.id = 0;
    current_unknowns.type = visualization_msgs::Marker::SPHERE_LIST;
    current_unknowns.scale.x = 0.4;
    current_unknowns.scale.y = 0.4;
    current_unknowns.scale.z = 0.4;
    current_unknowns.color.b = 1;
    current_unknowns.color.a = 0.8;
    current_unknown_line_list.ns = "lines";
    current_unknown_line_list.id = 1;
    current_unknown_line_list.type = visualization_msgs::Marker::LINE_LIST;
    current_unknown_line_list.scale.x = 0.1;
    // GRAPH_line_list.color.r = 0.8;
    current_unknown_line_list.color.g = 0.8;
    current_unknown_line_list.color.b = 0.2;
    current_unknown_line_list.color.a = 0.8;

    // for (auto it_c = current.unknowns.begin(); it_c != current.unknowns.end(); it_c++)
    // {
    //   geometry_msgs::Point unknown;
    //   unknown.x = (*it_c).x();
    //   unknown.y = (*it_c).y();
    //   unknown.z = (*it_c).z();
    //   current_unknowns.points.push_back(unknown);

    // }

    std::list<node *>::iterator it_vis;
    for (it_vis = skeleton.begin(); it_vis != skeleton.end(); it_vis++)
    {
      geometry_msgs::Point p;
      p.x = (*it_vis)->point->x();
      p.y = (*it_vis)->point->y();
      p.z = (*it_vis)->point->z();
      current_unknowns.points.push_back(p);
      if ((*it_vis)->myParent != nullptr)
      {
        current_unknown_line_list.points.push_back(p);
        p.x = (*it_vis)->myParent->point->x();
        p.y = (*it_vis)->myParent->point->y();
        p.z = (*it_vis)->myParent->point->z();
        current_unknown_line_list.points.push_back(p);
      }
    }

    pub.current_unknowns_visualization_pub.publish(current_unknowns);
    pub.current_unknowns_visualization_pub.publish(current_unknown_line_list);
  }

  if (!local.unknowns.empty())
  {
    local_unknowns.header.frame_id = ODOM_FRAME_ID;
    local_unknowns.ns = "local";
    local_unknowns.action = visualization_msgs::Marker::ADD;
    local_unknowns.pose.orientation.w = 1.0;
    local_unknowns.id = 1;
    local_unknowns.type = visualization_msgs::Marker::SPHERE_LIST;
    local_unknowns.scale.x = 0.44;
    local_unknowns.scale.y = 0.44;
    local_unknowns.scale.z = 0.44;
    local_unknowns.color.r = 0.0;
    local_unknowns.color.g = 1;
    local_unknowns.color.b = 0.0;
    local_unknowns.color.a = 1;
    
    for (auto it_c = local.unknowns.begin(); it_c != local.unknowns.end(); it_c++)
    {
      geometry_msgs::Point unknown;
      unknown.x = (*it_c).x();
      unknown.y = (*it_c).y();
      unknown.z = (*it_c).z();
      local_unknowns.points.push_back(unknown);

    }
    pub.current_unknowns_visualization_pub.publish(local_unknowns);
  }

    if (!global.unknowns.empty())
  {
    global_unknowns.header.frame_id = ODOM_FRAME_ID;
    global_unknowns.ns = "global";
    global_unknowns.action = visualization_msgs::Marker::ADD;
    global_unknowns.pose.orientation.w = 1.0;
    global_unknowns.id = 2;
    global_unknowns.type = visualization_msgs::Marker::SPHERE_LIST;
    global_unknowns.scale.x = 0.44;
    global_unknowns.scale.y = 0.44;
    global_unknowns.scale.z = 0.44;
    global_unknowns.color.r = 0.6;
    global_unknowns.color.g = 0;
    global_unknowns.color.b = 0;
    global_unknowns.color.a = 1;
    
    for (auto it_c = global.unknowns.begin(); it_c != global.unknowns.end(); it_c++)
    {
      geometry_msgs::Point unknown;
      unknown.x = (*it_c).x();
      unknown.y = (*it_c).y();
      unknown.z = (*it_c).z();
      global_unknowns.points.push_back(unknown);

    }
    pub.current_unknowns_visualization_pub.publish(global_unknowns);
  }

  CURRENT_GOAL.header.frame_id = ODOM_FRAME_ID;
  CURRENT_GOAL.ns = "goal";
  CURRENT_GOAL.action = visualization_msgs::Marker::ADD;
  CURRENT_GOAL.pose.position.x = current.end_pt.x();
  CURRENT_GOAL.pose.position.y = current.end_pt.y();
  CURRENT_GOAL.pose.position.z = current.end_pt.z();
  CURRENT_GOAL.pose.orientation.x = 0;
  CURRENT_GOAL.pose.orientation.y = 0;
  CURRENT_GOAL.pose.orientation.z = 0;
  CURRENT_GOAL.pose.orientation.w = 1;
  CURRENT_GOAL.id = 0;
  CURRENT_GOAL.type = visualization_msgs::Marker::SPHERE;
  CURRENT_GOAL.scale.x = 2.5;
  CURRENT_GOAL.scale.y = 2.5;
  CURRENT_GOAL.scale.z = 2.5;
  CURRENT_GOAL.color.r = 0.58;
  CURRENT_GOAL.color.g = 0.77;
  CURRENT_GOAL.color.b = 0.45;
  CURRENT_GOAL.color.a = 0.6;
  pub.current_goal_vis_pub.publish(CURRENT_GOAL);

  if (!Local_Planning_Graph.empty())
  {
    GRAPH_nodes.header.frame_id = GRAPH_line_list.header.frame_id = ODOM_FRAME_ID;
    GRAPH_nodes.ns = "points";
    GRAPH_line_list.ns = "lines";
    GRAPH_nodes.action = visualization_msgs::Marker::ADD;
    GRAPH_nodes.pose.orientation.w = 1.0;
    GRAPH_nodes.id = 0;
    GRAPH_line_list.id = 1;
    GRAPH_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    GRAPH_line_list.type = visualization_msgs::Marker::LINE_LIST;
    GRAPH_nodes.scale.x = 0.4;
    GRAPH_nodes.scale.y = 0.4;
    GRAPH_nodes.scale.z = 0.4;
    GRAPH_line_list.scale.x = 0.02;
    GRAPH_line_list.scale.y = 0.02;
    GRAPH_nodes.color.r = 0.6;
    // GRAPH_nodes.color.g = 0.8;
    // GRAPH_nodes.color.b = 0.8;
    GRAPH_nodes.color.a = 1;
    GRAPH_line_list.color.r = 0.5;
    GRAPH_line_list.color.g = 0.5;
    GRAPH_line_list.color.b = 0.5;
    GRAPH_line_list.color.a = 1;
    std::list<node *>::iterator it_vis;
    for (it_vis = Local_Planning_Graph.begin(); it_vis != Local_Planning_Graph.end(); it_vis++)
    {
      geometry_msgs::Point p;
      p.x = (*it_vis)->point->x();
      p.y = (*it_vis)->point->y();
      p.z = (*it_vis)->point->z();
      GRAPH_nodes.points.push_back(p);
      if ((*it_vis)->myParent != nullptr)
      {
        GRAPH_line_list.points.push_back(p);
        p.x = (*it_vis)->myParent->point->x();
        p.y = (*it_vis)->myParent->point->y();
        p.z = (*it_vis)->myParent->point->z();
        GRAPH_line_list.points.push_back(p);
      }
    }
    pub.points_pub.publish(GRAPH_nodes);
    pub.points_pub.publish(GRAPH_line_list);
  }

  if (!ALC_POINTS.empty())
  {
    ALC_points.header.frame_id = ALC_line_list.header.frame_id = ODOM_FRAME_ID;
    ALC_points.ns = "points";
    ALC_points.action = visualization_msgs::Marker::ADD;
    ALC_points.pose.orientation.w = 1.0;
    ALC_points.id = 0;
    ALC_line_list.id = 1;
    ALC_points.type = visualization_msgs::Marker::POINTS;
    ALC_line_list.type = visualization_msgs::Marker::LINE_LIST;
    ALC_points.scale.x = 0.2;
    ALC_points.scale.y = 0.2;
    ALC_line_list.scale.x = 0.1;
    ALC_points.color.r = 0.8f;
    ALC_points.color.b = 0.8f;
    ALC_points.color.a = 1.0;
    ALC_line_list.color.b = 1.0;
    ALC_line_list.color.a = 1.0;
    std::list<node *>::iterator it_vis9;
    for (it_vis9 = ALC_POINTS.begin(); it_vis9 != ALC_POINTS.end(); it_vis9++)
    {
      geometry_msgs::Point point1;
      point1.x = (*it_vis9)->point->x();
      point1.y = (*it_vis9)->point->y();
      point1.z = (*it_vis9)->point->z();
      ALC_points.points.push_back(point1);
      // geometry_msgs::Point point2;
      // point2.x = (*(std::next(it_vis9, 1)))->point->x();
      // point2.y = (*(std::next(it_vis9, 1)))->point->y();
      // point2.z = (*(std::next(it_vis9, 1)))->point->z();
      // ALC_line_list.points.push_back(point1);
      // ALC_line_list.points.push_back(point2);
    }
    pub.alc_points_visualization_pub.publish(ALC_points);
    // pub.alc_path_visualization_pub.publish(ALC_line_list);
  }

  if (!traversable.list.empty())
  {
    TRAVERSABLE_points.header.frame_id = ODOM_FRAME_ID;
    TRAVERSABLE_points.ns = "points";
    TRAVERSABLE_points.action = visualization_msgs::Marker::ADD;
    TRAVERSABLE_points.pose.orientation.w = 1.0;
    TRAVERSABLE_points.id = 0;
    TRAVERSABLE_points.type = visualization_msgs::Marker::SPHERE_LIST;
    TRAVERSABLE_points.scale.x = 0.6;
    TRAVERSABLE_points.color.r = 0.0f;
    TRAVERSABLE_points.color.g = 1.0f;
    TRAVERSABLE_points.color.b = 1.0f;
    TRAVERSABLE_points.color.a = 1.0;
    std::list<ufo::math::Vector3>::iterator it_vis2;
    for (it_vis2 = traversable.list.begin(); it_vis2 != traversable.list.end(); it_vis2++)
    {
      geometry_msgs::Point p;
      p.x = it_vis2->x();
      p.y = it_vis2->y();
      p.z = it_vis2->z();
      TRAVERSABLE_points.points.push_back(p);
    }
    pub.hits_pub.publish(TRAVERSABLE_points);
  }

  if (!local.list.empty())
  {
    local_frontier_nodes.header.frame_id = ODOM_FRAME_ID;
    local_frontier_nodes.ns = "points";
    local_frontier_nodes.action = visualization_msgs::Marker::ADD;
    local_frontier_nodes.pose.orientation.w = 1.0;
    local_frontier_nodes.id = 0;
    local_frontier_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    local_frontier_nodes.scale.x = 0.5;
    local_frontier_nodes.scale.y = 0.5;
    local_frontier_nodes.scale.z = 0.5;
    local_frontier_nodes.color.g = 0.7f;
    local_frontier_nodes.color.a = 0.6;
    std::list<ufo::math::Vector3>::iterator it_vis3;
    for (it_vis3 = local.list.begin(); it_vis3 != local.list.end(); it_vis3++)
    {
      geometry_msgs::Point p;
      p.x = it_vis3->x();
      p.y = it_vis3->y();
      p.z = it_vis3->z();
      local_frontier_nodes.points.push_back(p);
    }
    pub.local_nodes_pub.publish(local_frontier_nodes);
  }

  if (!global.list.empty())
  {
    global_frontier_nodes.header.frame_id = ODOM_FRAME_ID;
    global_frontier_nodes.ns = "points";
    global_frontier_nodes.action = visualization_msgs::Marker::ADD;
    global_frontier_nodes.pose.orientation.w = 1.0;
    global_frontier_nodes.id = 0;
    global_frontier_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    global_frontier_nodes.scale.x = 0.4;
    global_frontier_nodes.color.b = 0.8f;
    global_frontier_nodes.color.r = 0.8f;
    global_frontier_nodes.color.g = 0.8f;
    global_frontier_nodes.color.a = 0.6;
    std::list<ufo::math::Vector3>::iterator it_vis5;
    for (it_vis5 = global.list.begin(); it_vis5 != global.list.end(); it_vis5++)
    {
      geometry_msgs::Point p0;
      p0.x = it_vis5->x();
      p0.y = it_vis5->y();
      p0.z = it_vis5->z();
      global_frontier_nodes.points.push_back(p0);
    }
    pub.global_nodes_pub.publish(global_frontier_nodes);
  }

  if (!old_poses.list.empty())
  {
    nav_msgs::Path exploration_path;
    for (auto visited_it = old_poses.list.begin(); visited_it != old_poses.list.end(); visited_it++)
    {
      geometry_msgs::PoseStamped past_pose;
      past_pose.header.stamp = ros::Time::now();
      past_pose.header.frame_id = ODOM_FRAME_ID;
      past_pose.pose.position.x = visited_it->x();
      past_pose.pose.position.y = visited_it->y();
      past_pose.pose.position.z = visited_it->z();
      past_pose.pose.orientation.x = 0;
      past_pose.pose.orientation.y = 0;
      past_pose.pose.orientation.z = 0;
      past_pose.pose.orientation.w = 0;
      exploration_path.poses.push_back(past_pose);
    }
    exploration_path.header.stamp = ros::Time::now();
    exploration_path.header.frame_id = ODOM_FRAME_ID;
    pub.explored_path.publish(exploration_path);
  }
}

void fetchTraversable(std::list<ufo::math::Vector3> frontier_list)
{
  std::list<ufo::math::Vector3> extracted;
  for (ufo::math::Vector3 goal : frontier_list)
  {
    ufo::geometry::OBB obb = makeOBB (current.position, goal, 0.8);                       
    if (!isInCollision(ufoMAP, obb, true, false, false, PLANNING_CHECK_DEPTH) && calculateDistance(goal, current.position) > 3)
    {
      extracted.push_back(goal);
    }
  }
  traversable.list = extracted;
}

double calculateAngle(ufo::math::Vector3 fpoint, ufo::math::Vector3 position,
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

bool isLocal(ufo::math::Vector3 front)
{
  double angle = calculateAngle(front, current.position, current.heading);
  double distance = calculateDistance(front, current.position);
  if (((angle < 40) && (angle > -40)) && distance < LOCAL_RANGE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

ufo::math::Vector3 informativeFrontier()
{
  ufo::math::Vector3 most_info_frontier;
  double radius = 0.1;
  for (ufo::math::Vector3 f_point : local.list)
  {

    ufo::geometry::Sphere sphere(f_point, radius);
    if (isInCollision(ufoMAP, sphere, false, true, false, PLANNING_CHECK_DEPTH) && !isInCollision(ufoMAP, sphere, true, false, true, PLANNING_CHECK_DEPTH))
    {
      radius = radius + 0.1;
      most_info_frontier = f_point;
    }
  }
  return most_info_frontier;
}

ufo::math::Vector3 selectLocalFrontier(std::list<ufo::math::Vector3> accessible,
                                       ufo::math::Vector3 position, double heading)
{
  double cost;
  double mycost = 100000000;
  ufo::math::Vector3 select;
  for (ufo::math::Vector3 localfrontier : accessible)
  {
    double distance = calculateDistance(localfrontier, position);
    double angle = calculateAngle(localfrontier, position, heading);
    cost = angle;
    if ((localfrontier.z() > 3) || (localfrontier.z() < 0.8))
    {
      cost = cost + 10000;
    }
    else
    {
      cost = cost;
    }
    if (cost < 0)
    {
      cost = -cost;
      if (cost < mycost)
      {
        mycost = cost;
        select = localfrontier;
      }
    }
    if (cost > 0)
    {
      if (cost < mycost)
      {
        mycost = cost;
        select = localfrontier;
      }
    }
  }
  return select;
}

node *temp_selectGlobalFrontier(std::list<ufo::math::Vector3> global)
{
  double score;
  double myScore = 100000000;
  ufo::math::Vector3 selectg;

  for (ufo::math::Vector3 globalfrontier : global)
  {
    score = calculateDistance(globalfrontier, current.position);
    if (score < myScore)
    {
      myScore = score;
      selectg = globalfrontier;
    }
  }
  double pen;
  double mypen = 10000000000000;
  node *my_nod;
  for (auto rit = Global_Graph.begin(); rit != Global_Graph.end(); rit++)
  {

    double nearest_dis = calculateDistance(*((*rit)->point), selectg);
    pen = nearest_dis;
    if (pen < mypen)
    {
    double radius = 1;
    ufo::geometry::Sphere sphere(*((*rit)->point), radius);
    if (isInCollision(ufoMAP, sphere, false, true, false, PLANNING_CHECK_DEPTH) && !isInCollision(ufoMAP, sphere, true, false, true, PLANNING_CHECK_DEPTH))
    {
      mypen = pen;
      my_nod = *rit;
    }
    }
  }
  return my_nod;
}

ufo::math::Vector3 selectGlobalFrontier(std::list<ufo::math::Vector3> global)
{
  double score;
  double myScore = 100000000;
  ufo::math::Vector3 selectg;

  for (ufo::math::Vector3 globalfrontier : global)
  {
    score = calculateDistance(globalfrontier, current.position);
    if (score < myScore)
    {
      myScore = score;
      selectg = globalfrontier;
    }
  }

  return selectg;
}

node *switchedGlobalFrontier(std::list<ufo::math::Vector3> global)
{
  double score;
  double myScore = 0;
  ufo::math::Vector3 selectg;

  for (ufo::math::Vector3 globalfrontier : global)
  {
    score = calculateDistance(globalfrontier, current.position);
    if (score > myScore)
    {
      myScore = score;
      selectg = globalfrontier;
    }
  }
  double pen;
  double mypen = 10000000000000;
  node *my_nod;
  for (auto rit = Global_Graph.begin(); rit != Global_Graph.end(); rit++)
  {

    double nearest_dis = calculateDistance(*((*rit)->point), selectg);
    pen = nearest_dis;
    if (pen < mypen)
    {
    double radius = 1;
    ufo::geometry::Sphere sphere(*((*rit)->point), radius);
    if (isInCollision(ufoMAP, sphere, false, true, false, PLANNING_CHECK_DEPTH) && !isInCollision(ufoMAP, sphere, true, false, true, PLANNING_CHECK_DEPTH))
    {
      mypen = pen;
      my_nod = *rit;
    }

    }
  }
  return my_nod;
}

std::list<node *> LoopClosurePath(std::list<ufo::math::Vector3> global)
{
  double score1;
  double myScore1 = 100000000;
  ufo::math::Vector3 selectg1;
  std::list<node *> alc_candidates;
  std::list<ufo::math::Vector3> successful_ray;

  for (ufo::math::Vector3 globalfrontier1 : global)
  {
    score1 = calculateDistance(globalfrontier1, current.position);
    if (score1 < myScore1)
    {
      myScore1 = score1;
      selectg1 = globalfrontier1;
    }
  }

  for (auto old_poses_it = old_poses.list.begin(); old_poses_it != old_poses.list.end(); old_poses_it++)
  {

    ufo::math::Vector3 start_pt((*old_poses_it).x(), (*old_poses_it).y(), (*old_poses_it).z());
    ufo::geometry::LineSegment myLine(selectg1, start_pt);
    if (!isInCollision(ufoMAP, myLine, true, false, true, PLANNING_CHECK_DEPTH))
    {
      successful_ray.push_back(*old_poses_it);
    }
  }

  for (auto tree_it = Global_Graph.begin(); tree_it != Global_Graph.end(); tree_it++)
  {

    for (auto ray_it = successful_ray.begin(); ray_it != successful_ray.end(); ray_it++)
    {

      double d = calculateDistance(*ray_it, *((*tree_it)->point));

      if (d < 0.2)
      {
        alc_candidates.push_back(*tree_it);
      }
    }
  }

  return alc_candidates;
}

bool next_pub()
{

  ufo::math::Vector3 pos;
  pos.x() = published.reference.pose.position.x;
  pos.y() = published.reference.pose.position.y;
  pos.z() = published.reference.pose.position.z;
  double pub_dis = calculateDistance(pos, current.position);
  double angle = calculateAngle(pos, current.position, current.heading);

  if (pub_dis < NEXT_PUB_DISTANCE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

geometry_msgs::PoseStamped publish_goal(ufo::math::Vector3 selected)
{
  geometry_msgs::PoseStamped pub_frontier;
  pub_frontier.pose.position.x = selected.x();
  pub_frontier.pose.position.y = selected.y();
  pub_frontier.pose.position.z = selected.z();
  pub_frontier.pose.orientation.x = 0;
  pub_frontier.pose.orientation.y = 0;
  pub_frontier.pose.orientation.z = atan2((selected.y() - current.position.y()), (selected.x() - current.position.x()));
  ;
  pub_frontier.pose.orientation.w = 0;
  return pub_frontier;
}
//--------------------------------------------------  CALL BACK FUNCTIONS --------------------------------------------//
void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg)
{
  if (ufomap_msgs::msgToUfo(msg->map, ufoMAP))
  {
    map_received = true;
  }
  else
  {
    ROS_INFO("Map conversion failed");
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  position_received = true;
  ufo::math::Vector3 oldPoint;
  ufo::math::Vector3 newPoint;
  position_x = msg->pose.pose.position.x;
  position_y = msg->pose.pose.position.y;
  position_z = msg->pose.pose.position.z;
  velocity_x = msg->twist.twist.linear.x;
  velocity_y = msg->twist.twist.linear.y;
  velocity_z = msg->twist.twist.linear.z;
  if (start_flag)
  {
    m_prevPose = *msg;
    start_flag = false;
  }
  else
  {
    m_currentPose = *msg;
    newPoint = ufo::math::Vector3(msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  msg->pose.pose.position.z);
    oldPoint = ufo::math::Vector3(m_prevPose.pose.pose.position.x,
                                  m_prevPose.pose.pose.position.y,
                                  m_prevPose.pose.pose.position.z);
    if (calculateDistance(newPoint, oldPoint) > 0.5)
    {
      m_prevPose = *msg;
      positionslist.push_back(oldPoint);
    }
    if (calculateDistance(newPoint, oldPoint) > 1.6)
    {
      move_flag = true;
    }
    else
    {
      move_flag = false;
    }
    old_poses.list = positionslist;
  }
  current.position = ufo::math::Vector3(position_x, position_y, position_z);
  current.pose.pose.position.x = position_x;
  current.pose.pose.position.y = position_y;
  current.pose.pose.position.z = position_z;
  current.pose.pose.orientation.x = 0;
  current.pose.pose.orientation.y = 0;
  current.pose.pose.orientation.z = 0;
  current.pose.pose.orientation.w = 0;
  tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  current.heading = yaw * (180 / 3.14159);

  visualization_msgs::Marker ROBOT_BOX;
  ROBOT_BOX.header.frame_id = ODOM_FRAME_ID;
  ROBOT_BOX.ns = "points";
  ROBOT_BOX.action = visualization_msgs::Marker::ADD;
  ROBOT_BOX.pose.position.x = current.position.x();
  ROBOT_BOX.pose.position.y = current.position.y();
  ROBOT_BOX.pose.position.z = current.position.z();
  ROBOT_BOX.pose.orientation.x = 0;
  ROBOT_BOX.pose.orientation.y = 0;
  ROBOT_BOX.pose.orientation.z = 0;
  ROBOT_BOX.pose.orientation.w = 1;
  ROBOT_BOX.id = 0;
  ROBOT_BOX.type = visualization_msgs::Marker::CUBE;
  ROBOT_BOX.scale.x = 4;
  ROBOT_BOX.scale.y = 4;
  ROBOT_BOX.scale.z = 4;
  ROBOT_BOX.color.r = 1.0;
  ROBOT_BOX.color.a = 0.2;
  pub.robot_box_vis_pub.publish(ROBOT_BOX);

}

nav_msgs::Path backTrack(bool backtrack)
{

  nav_msgs::Path retrace;
  retrace.poses.push_back(current.pose);
  if (backtrack)
  {

    for (auto backtrack_it = old_poses.list.rbegin(); backtrack_it != (old_poses.list.rend()); backtrack_it++)
    {
      
      geometry_msgs::PoseStamped global_nodes2;

      global_nodes2.pose.position.x = backtrack_it->x();
      global_nodes2.pose.position.y = backtrack_it->y();
      global_nodes2.pose.position.z = backtrack_it->z();
      // std::cout<<"backtrace adding"<<std::endl;
      // std::cout<<backtrack_it->x()<<" "<<backtrack_it->y()<<" "<<backtrack_it->z()<<std::endl;

      global_nodes2.header.stamp = ros::Time::now();
      global_nodes2.header.frame_id = ODOM_FRAME_ID;
      retrace.poses.push_back(global_nodes2);
      ufo::geometry::OBB obb = makeOBB(*backtrack_it, current.end_pt, 0.5);
      if (!isInCollision(ufoMAP, obb, true, false, true, PLANNING_CHECK_DEPTH)) {
        global_nodes2.pose.position.x = current.end_pt.x();
        global_nodes2.pose.position.y = current.end_pt.y();
        global_nodes2.pose.position.z = current.end_pt.z();
        retrace.poses.push_back(global_nodes2);
        published.reference = publish_goal(current.end_pt);
        break;
      }
    }
  }

  start_backtrack.navigate = false;

  return retrace;
}

void homingCallback(const std_msgs::String::ConstPtr &home_coming)
{

  home.navigate = true;
  explore.navigate = false;
}

void exploreCallback(const std_msgs::String::ConstPtr &begin_explore)
{

  explore.navigate = true;
  home.navigate = false;
}

void switchCallback(const std_msgs::String::ConstPtr &go_global)
{

  switch_global.navigate = true;
  temp_local.navigate = false;
}

void frontiersCallback(const acl_exploration::Frontiers::ConstPtr &msg)
{

  frontiers_received = true;
  ufo::math::Vector3 frontier_point;
  std::list<ufo::math::Vector3> seg1, seg2, seg3, seg4, seg5, seg6, 
                                      seg7, seg8, myseg, mylist, mylist1, mylist2;

  for (geometry_msgs::Point frontier : msg->points)
  {
    frontier_point = ufo::math::Vector3(frontier.x, frontier.y, frontier.z);
    ufo::geometry::Sphere sphere(frontier_point, RAW_FRONTIERS_COLISSION_MARGIN);
    if (!isInCollision(ufoMAP, sphere, true, false, true, PLANNING_CHECK_DEPTH))
    {
      mylist.push_back(frontier_point);
      all.list = mylist;
      if (isLocal(frontier_point))
      {
        mylist1.push_back(frontier_point);
        local.list = mylist1;
      }
      else
      {
        mylist2.push_back(frontier_point);
        global.list = mylist2;
      }
    }
  }

  for (ufo::math::Vector3 localnode : local.list)
  {

    double alpha = calculateAngle(localnode, current.position, current.heading);

    if ((alpha < -30) && (alpha > -40))
    {
      seg1.push_back(localnode);
    }
    if ((alpha < -20) && (alpha > -30))
    {
      seg2.push_back(localnode);
    }
    if ((alpha < -10) && (alpha > -20))
    {
      seg3.push_back(localnode);
    }
    if ((alpha < 0) && (alpha > -10))
    {
      seg4.push_back(localnode);
    }
    if ((alpha < 10) && (alpha > 0))
    {
      seg5.push_back(localnode);
    }
    if ((alpha < 20) && (alpha > 10))
    {
      seg6.push_back(localnode);
    }
    if ((alpha < 30) && (alpha > 20))
    {
      seg7.push_back(localnode);
    }
    if ((alpha < 40) && (alpha > 30))
    {
      seg8.push_back(localnode);
    }

    if (seg1.size() > myseg.size())
    {
      myseg = seg1;
    }
    if (seg2.size() > myseg.size())
    {
      myseg = seg2;
    }
    if (seg3.size() > myseg.size())
    {
      myseg = seg3;
    }
    if (seg4.size() > myseg.size())
    {
      myseg = seg4;
    }
    if (seg5.size() > myseg.size())
    {
      myseg = seg5;
    }
    if (seg6.size() > myseg.size())
    {
      myseg = seg6;
    }
    if (seg7.size() > myseg.size())
    {
      myseg = seg7;
    }
    if (seg8.size() > myseg.size())
    {
      myseg = seg8;
    }

    max_information.list = myseg;
  }

  fetchTraversable(max_information.list);

  if (!traversable.list.empty())
  {
    local.navigate = true;
  }
  else
  {
    local.navigate = false;
  }

  if (traversable.list.empty() and !all.list.empty())
  {
    global.navigate = true;
  }
  else
  {
    global.navigate = false;
  }

  if (msg->points.empty())
  {
    home.navigate = true;
  }
}

void fetchUnknownNodes(ufo::math::Vector3 point, 
                                ufo::map::OccupancyMapColor const& map){
  
  // current.unknowns.clear();
  // skeleton.clear();

      // Setting up targets, X / Y targets in either positive (P) or negative (N) direction
      ufo::math::Vector3 targetXP(point.x() + 1, point.y(), point.z());
      ufo::math::Vector3 targetXN(point.x() - 1, point.y(), point.z());
      ufo::math::Vector3 targetYP(point.x(), point.y() + 1, point.z());
      ufo::math::Vector3 targetYN(point.x(), point.y() - 1, point.z());
      ufo::math::Vector3 upwards(point.x(), point.y(), point.z() + 1);
    
      //Angles
      double vertical_angle = 0.275;
      double horizontal_angle = 3.14;
    
      //Distances
      double near_distance = 0.5;
      double far_distance = 16;
    
      //Frustums
      ufo::geometry::Frustum frustXP(point, targetXP, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
      ufo::geometry::Frustum frustXN(point, targetXN, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
      ufo::geometry::Frustum frustYP(point, targetYP, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
      ufo::geometry::Frustum frustYN(point, targetYN, upwards, vertical_angle, horizontal_angle, near_distance, far_distance);
      int checks = 0;
      for (auto it = map.beginLeaves(frustXP, false, false, true, false, PLANNING_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it){
        ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
        ufo::geometry::OBB obb = makeOBB(point, end_point, 0.4);
        if (abs(it.getZ() - point.z()) < 2) {

          ufo::geometry::LineSegment myLine(point, end_point);
          checks++;
          if(!isInCollision(map, obb, true, false, false, PLANNING_CHECK_DEPTH) ){

            current.unknowns.push_back(end_point);
          }
        }
      }
      for (auto it = map.beginLeaves(frustXN, false, false, true, false, PLANNING_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it){
        ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
        ufo::geometry::OBB obb = makeOBB(point, end_point, 0.4);
        if (abs(it.getZ() - point.z()) < 1.5) {

          ufo::geometry::LineSegment myLine(point, end_point);
          checks++;
          if(!isInCollision(map, obb, true, false, false, PLANNING_CHECK_DEPTH) ){

            current.unknowns.push_back(end_point);
          }
        }
      }
      for (auto it = map.beginLeaves(frustYP, false, false, true, false, PLANNING_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it){
        ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
        ufo::geometry::OBB obb = makeOBB(point, end_point, 0.4);
        if (abs(it.getZ() - point.z()) < 1.5) {

          ufo::geometry::LineSegment myLine(point, end_point);
          checks++;
          if(!isInCollision(map, obb, true, false, false, PLANNING_CHECK_DEPTH) ){

            current.unknowns.push_back(end_point);
          }
        }
      }
      for (auto it = map.beginLeaves(frustYN, false, false, true, false, PLANNING_CHECK_DEPTH), it_end = map.endLeaves(); it != it_end; ++it){
        ufo::math::Vector3 end_point(it.getX(), it.getY(), it.getZ());
        ufo::geometry::OBB obb = makeOBB(point, end_point, 0.4);
        if ( abs(it.getZ() - point.z()) < 1.5) {

          ufo::geometry::LineSegment myLine(point, end_point);
          checks++;
          if(!isInCollision(map, obb, true, false, false, PLANNING_CHECK_DEPTH) ){

            current.unknowns.push_back(end_point);
          }
        }
      }

      node *pos_parent = new node(point.x(),point.y(),point.z());

      for (auto it = current.unknowns.begin(); it != current.unknowns.end(); it++) {

        node *robot_pose = new node(it->x(), it->y(), it->z());
        robot_pose->addParent(pos_parent);
        pos_parent->addChild(robot_pose);
        skeleton.push_back(robot_pose); 

      }

      // for (auto clear_it = skeleton.begin(); clear_it != skeleton.end(); clear_it++) {

      //   // ufo::geometry::OBB obb = makeOBB(*((*clear_it)->myParent->point),*((*clear_it)->point) , 0.4);
      //   ufo::geometry::Sphere sphere(*((*clear_it)->point), 2);
      //   if(isInCollision(map, sphere, false, true, false, PLANNING_CHECK_DEPTH) or isInCollision(map, sphere, true, false, false, PLANNING_CHECK_DEPTH)){
      //     clear_it = skeleton.erase(clear_it);
      //   }
      // }

      std::list<ufo::math::Vector3>::iterator it_unkwn;	
      for(it_unkwn = current.unknowns.begin(); it_unkwn != current.unknowns.end(); it_unkwn++){
        // ufo::geometry::LineSegment myLine2(point, *it_unkwn);
        ufo::geometry::OBB obb = makeOBB(point, *it_unkwn, 0.5);
        if(isInCollision(map, obb, true, false, false, PLANNING_CHECK_DEPTH)){
          it_unkwn = current.unknowns.erase(it_unkwn);
        }
        else{
          ufo::geometry::Sphere sphere(*it_unkwn, 2);
          if(isInCollision(map, sphere, false, true, false, PLANNING_CHECK_DEPTH)){
            it_unkwn = current.unknowns.erase(it_unkwn);
          }
        }
      }

      for(it_unkwn = local.unknowns.begin(); it_unkwn != local.unknowns.end(); it_unkwn++){
        // ufo::geometry::LineSegment myLine2(point, *it_unkwn);
        ufo::geometry::OBB obb = makeOBB(point, *it_unkwn, 0.5);
        if(isInCollision(map, obb, true, false, false, PLANNING_CHECK_DEPTH)){
          it_unkwn = local.unknowns.erase(it_unkwn);
        }
        else{
          ufo::geometry::Sphere sphere(*it_unkwn, 2);
          if(isInCollision(map, sphere, false, true, false, PLANNING_CHECK_DEPTH)){
            it_unkwn = local.unknowns.erase(it_unkwn);
          }
        }
      }
      for(it_unkwn = global.unknowns.begin(); it_unkwn != global.unknowns.end(); it_unkwn++){
        // ufo::geometry::LineSegment myLine2(point, *it_unkwn);
        ufo::geometry::Sphere sphere(*it_unkwn, 2);
        ufo::geometry::Sphere sphere2(*it_unkwn, 0.5);

        ufo::geometry::OBB obb = makeOBB(point, *it_unkwn, 0.5);
          if(isInCollision(map, sphere, true, false, false, PLANNING_CHECK_DEPTH)){
            it_unkwn = global.unknowns.erase(it_unkwn);
          } 
          // else {
          // if(isInCollision(map, sphere2, false, true, false, PLANNING_CHECK_DEPTH)){
          //   it_unkwn = global.unknowns.erase(it_unkwn);
          // }
          // }
      }

    for (ufo::math::Vector3 unknown_node : current.unknowns) {

      if (isLocal(unknown_node)) {

        local.unknowns.push_back(unknown_node);
      } else {
          global.unknowns.push_back(unknown_node);
      }

    }

}

double lookAround (ufo::math::Vector3 final_pose) {
  cout << "looking at .." << endl;
  lookAt.list.clear();
  if (!all.list.empty()) {
    for (ufo::math::Vector3 frontier : local.list) {

      double dist = calculateDistance(frontier, current.position);
      if (dist < LOCAL_RANGE) {
        ufo::geometry::LineSegment line_seg (current.position, frontier);
        ufo::geometry::OBB obb = makeOBB(current.position, frontier, 0.5);
        if (!isInCollision(ufoMAP, obb, true, false, true, PLANNING_CHECK_DEPTH)) {
          lookAt.list.push_back(frontier);
        }
      }
    }
  }
  cout << "\nALL " << all.list.size() << endl;
  cout << "\nLOOK AT -----  " << lookAt.list.size() << endl;

  ufo::math::Vector3 look_towards_frontier;
  if (!lookAt.list.empty()) {
    
    double radius = 0.4;
    for (ufo::math::Vector3 f_point : lookAt.list)
    {
      ufo::geometry::Sphere sphere(f_point, radius);
      if (isInCollision(ufoMAP, sphere, false, true, false, PLANNING_CHECK_DEPTH) && 
                                          !isInCollision(ufoMAP, sphere, true, false, true, PLANNING_CHECK_DEPTH))
      {
        radius = radius + 0.1;
        look_towards_frontier = f_point;
      }
    }
  }
  
  if (look_towards_frontier.x() != 0 and look_towards_frontier.y() != 0) {
    cout << "found a frontier to look at " << endl;
    double head_ref = atan2(look_towards_frontier.y() - final_pose.y(), look_towards_frontier.x() - final_pose.x());
    // geometry_msgs::PoseStamped look_ref;
    // look_ref.pose.position.x = current.position.x();
    // look_ref.pose.position.y = current.position.y();
    // look_ref.pose.position.z = current.position.z();
    // look_ref.pose.orientation.w = 1;
    // look_ref.pose.orientation.x = 0;
    // look_ref.pose.orientation.y = 0;
    // look_ref.pose.orientation.z = atan2(look_towards_frontier.y() - current.position.y(), look_towards_frontier.x() - current.position.x());
    // lookAt.reference = look_ref;
    // pub.best_frontier_pub.publish(look_ref);
    // published.reference = look_ref;
    return head_ref;
  } 
}

void exploreLocally()
{

  // ufo::math::Vector3 traversable_goal = selectLocalFrontier(traversable.list, current.position, current.heading);
  ufo::math::Vector3 traversable_goal = informativeFrontier();
  if (traversable_goal.x() == 0 and traversable_goal.y() == 0)
  {
    traversable_goal = selectLocalFrontier(traversable.list, current.position, current.heading);
  }

  // -------------------------- graph based local exploration -------------------//

  double pen;
  double mypen = 10000000000000;
  node *local_nod;
  for (auto rit = Global_Graph.begin(); rit != Global_Graph.end(); rit++)
  {

    double nearest_dis = calculateDistance(*((*rit)->point), traversable_goal);
    pen = nearest_dis;
    if (pen < mypen)
    {

    double radius = 1;
    ufo::geometry::Sphere sphere(*((*rit)->point), radius);
    if (isInCollision(ufoMAP, sphere, false, true, false, PLANNING_CHECK_DEPTH) && !isInCollision(ufoMAP, sphere, true, false, true, PLANNING_CHECK_DEPTH))
    {
      mypen = pen;
      local_nod = *rit;
    }
    }
  }

  planning.graph_pointer = local_nod;
  cout << "Local path from Local planning graph" << endl;

  // -----------------------------------------------------------------------//

  // -------------------------- direct local exploration -------------------//

  // published.reference = publish_goal(traversable_goal);
  // current.temppath_local.poses.push_back(current.pose);
  // current.temppath_local.poses.push_back(published.reference);
  // current.path.path = current.temppath_local;
  // current.path.type.data = "local";
  // pub.path_to_goal_pub.publish(current.path);
  // pub.local_path_pub.publish(current.temppath_local);
  // current.path.path.poses.clear();
  // current.path.type.data.clear();
  // current.temppath_local.poses.clear();
  // cout << "Exploring Locally..." << endl;

  // -----------------------------------------------------------------------//
}

void fetchGlobalFrontier()
{

  double globalscore;
  double myglobalscore = 100000000;
  ufo::math::Vector3 gf;

  for (ufo::math::Vector3 globalfrontier : global.list)
  {
    globalscore = calculateDistance(globalfrontier, current.position);
    if (globalscore < myglobalscore)
    {
      myglobalscore = globalscore;
      gf = globalfrontier;
    }
  }
  global_frontier.position = gf;
}

void buildPlanningGraph(bool go_home, bool local_exploration, bool improvement, double reit, double range_up_scale)
{

  for (std::list<node *>::iterator it_clear_helper = Local_Planning_Graph.begin(); it_clear_helper != --Local_Planning_Graph.end(); it_clear_helper++)
  {
    (*it_clear_helper)->readyForDeletion();
    delete (*it_clear_helper);
  }

  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  int max_it;
  float planning_range;
  IMPROVEMENT_SCALE_FACTOR_IT = reit;
  std::vector<Graph::planning_node> my_nodes;
  std::vector<Graph::planning_node> parent_nodes;
  std::vector<Graph::planning_node> child_nodes;
  std::vector< std::pair< std::vector< Graph::planning_node >::iterator, std::vector < Graph::planning_node >::iterator > > PAIRS;

  PAIRS.clear();
  planning.node_container.clear();
  parent_nodes.clear();
  child_nodes.clear();
  Local_Planning_Graph.clear();
  if (go_home)
  {

    my_nodes.clear();

    for (auto insert_helper = old_poses.list.begin(); insert_helper != std::prev(old_poses.list.end(), 4); insert_helper++)
    {

      my_nodes.push_back({Graph::Point{insert_helper->x(), insert_helper->y(), insert_helper->z()}});
    }
  }

  if (!home.navigate and !go_home)
  {

    if (local_exploration)
    {

      int min = MIN_ITERATION_LOCAL;
      float original_planning_range = MIN_PLANNING_SPACE_LOCAL;

      if (improvement and max_it < MAX_ITERATION_LOCAL and planning_range < MAX_PLANNING_SPACE_LOCAL)
      {
        // if (improvement and max_it < MAX_ITERATION_LOCAL) {
        max_it = min + IMPROVEMENT_SCALE_FACTOR_IT * min;
        planning_range = original_planning_range + (range_up_scale * original_planning_range);
      }
      else
      {
        max_it = min;
        planning_range = MIN_PLANNING_SPACE_LOCAL;
      }
    }
    else
    {

      int min_G = MIN_ITERATION_GLOBAL;
      float original_planning_range_G = MIN_PLANNING_SPACE_GLOBAL;
      if (improvement and max_it < MAX_ITERATION_GLOBAL and planning_range < MAX_PLANNING_SPACE_GLOBAL)
      {
        // if (improvement and max_it < MAX_ITERATION_GLOBAL) {
        max_it = min_G + IMPROVEMENT_SCALE_FACTOR_IT * min_G;
        planning_range = original_planning_range_G + (range_up_scale * original_planning_range_G);
      }
      else
      {
        max_it = min_G;
        planning_range = MIN_PLANNING_SPACE_GLOBAL;
      }
    }

    current.end_pt.x() = 0;
    current.end_pt.y() = 0;
    current.end_pt.z() = 1;
    

    if (local_exploration)
    {
      current.end_pt = informativeFrontier();
    }
    else
    {
      current.end_pt = selectGlobalFrontier(all.list);
    }

    planningSpace(local_exploration, current.end_pt, planning_range, ufoMAP, false, true, false, current.position.x(), current.position.y(), current.position.z(), PLANNING_CHECK_DEPTH);
    buildGraph(current.position.x(), current.position.y(), current.position.z(), current.end_pt, max_it);

    if (!Global_Graph.empty())
    {
      for (auto insert_helper = Global_Graph.begin(); insert_helper != Global_Graph.end(); insert_helper++)
      {
        child_nodes.push_back({Graph::Point{(((*insert_helper)->point)->x()), (((*insert_helper)->point)->y()), (((*insert_helper)->point)->z())}});
        if ((*insert_helper)->myParent != nullptr) {
          parent_nodes.push_back({Graph::Point{((((*insert_helper))->myParent)->point)->x(), ((((*insert_helper))->myParent)->point)->y(),
                                         ((((*insert_helper))->myParent)->point)->z()}});
        }
      }
    }

    if (switch_global.navigate)
    {

      planning.graph_pointer = switchedGlobalFrontier(all.list);
    }
    else if (!home.navigate and !go_home)
    {

      planning.graph_pointer = temp_selectGlobalFrontier(all.list);
    }

    // ALC_POINTS = LoopClosurePath(all.list);    
    for (auto it = parent_nodes.begin(); it != --parent_nodes.end(); it++)
    {
      ufo::math::Vector3 p1((*it).myposition.x, (*it).myposition.y, (*it).myposition.z);

      for (auto it2 = it; it2 != parent_nodes.end(); it2++) {

        ufo::math::Vector3 p2((*it2).myposition.x, (*it2).myposition.y, (*it2).myposition.z);

        double dist = calculateDistance(p1,p2);
        if (dist > 2 and dist < 4) {
          PAIRS.push_back(std::make_pair(it, it2));
        }
      }
    }

    // cout << "pair check " << PAIRS.size() << endl;

    for (auto i = PAIRS.begin(); i != PAIRS.end(); i++) {

      ufo::math::Vector3 n1(((*i).first)->myposition.x, ((*i).first)->myposition.y, ((*i).first)->myposition.z);
      ufo::math::Vector3 n2(((*i).second)->myposition.x, ((*i).second)->myposition.y, ((*i).second)->myposition.z);
      ufo::geometry::OBB obb = makeOBB(n1, n2, PLANNING_GRAPH_COLISSION_MARGIN);

      if (local_exploration) {

        if (calculateDistance(n1, current.position) < LOCAL_RANGE and calculateDistance(n2, current.position) < LOCAL_RANGE) {
          if (isInCollision(ufoMAP, obb, false, true, false, PLANNING_CHECK_DEPTH) 
                  && !isInCollision(ufoMAP, obb, true, false, true, PLANNING_CHECK_DEPTH))
          {
            planning.node_container.push_back(*(i)->first);
            planning.node_container.push_back(*(i)->second);
            (*(i)->first).AddConnection(*(i)->second);
            (*(i)->second).AddConnection(*(i)->first);
            node *parent_node = new node(((*i).first)->myposition.x, ((*i).first)->myposition.y,
                                         ((*i).first)->myposition.z);
            node *child_node = new node(((*i).second)->myposition.x, ((*i).second)->myposition.y,
                                        ((*i).second)->myposition.z);
            child_node->addParent(parent_node);
            parent_node->addChild(child_node);
            Local_Planning_Graph.push_back(child_node);
          }
        }
      }
      else if (isInCollision(ufoMAP, obb, false, true, false, PLANNING_CHECK_DEPTH) 
                  && !isInCollision(ufoMAP, obb, true, false, true, PLANNING_CHECK_DEPTH))
      {
        planning.node_container.push_back(*(i)->first);
        planning.node_container.push_back(*(i)->second);
        (*(i)->first).AddConnection(*(i)->second);
        (*(i)->second).AddConnection(*(i)->first);
        node *parent_node = new node(((*i).first)->myposition.x, ((*i).first)->myposition.y,
                                     ((*i).first)->myposition.z);
        node *child_node = new node(((*i).second)->myposition.x, ((*i).second)->myposition.y,
                                    ((*i).second)->myposition.z);
        child_node->addParent(parent_node);
        parent_node->addChild(child_node);
        Local_Planning_Graph.push_back(child_node);
      }
    }
  } else if (home.navigate)
  {
    for (auto graph_it = my_nodes.begin(); graph_it != --my_nodes.end(); graph_it++)
    {
      ufo::math::Vector3 d1((*graph_it).myposition.x, (*graph_it).myposition.y, (*graph_it).myposition.z);

      for (auto graph_it2 = graph_it; graph_it2 != my_nodes.end(); graph_it2++)
      {
        ufo::math::Vector3 d2((*graph_it2).myposition.x, (*graph_it2).myposition.y, (*graph_it2).myposition.z);

        ufo::geometry::OBB obb = makeOBB (d2, d1, PLANNING_GRAPH_COLISSION_MARGIN);                       
        // ufo::geometry::LineSegment line_check(d1, d2);
        double rang = calculateDistance(d1, d2);

        if (rang > 1 and rang < 8)
        {
          if (!isInCollision(ufoMAP, obb, true, false, true, PLANNING_CHECK_DEPTH) &&
              isInCollision(ufoMAP, obb, false, true, false, PLANNING_CHECK_DEPTH))
          {
            planning.node_container.push_back(*graph_it2);
            planning.node_container.push_back(*graph_it);
            (*graph_it).AddConnection(*graph_it2);
            (*graph_it2).AddConnection(*graph_it);

          }
        }
      }
    }
  }

  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  // cout << "\n\033[;34m Making the planning graph takes... \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;
}

void findPath()
{

  high_resolution_clock::time_point start_total = high_resolution_clock::now();

  if (!home.navigate)
  {
    double close_dis = 1000000;
    double end_dis = 1000000;
    double my_end_dis = 100000000;
    double my_close_dis = 100000000;

    for (auto closest_it = planning.node_container.begin(); closest_it != planning.node_container.end(); closest_it++)
    {

      close_dis = std::sqrt(std::pow((*closest_it).myposition.x - current.position.x(), 2) +
                            std::pow((*closest_it).myposition.y - current.position.y(), 2) +
                            std::pow((*closest_it).myposition.z - current.position.z(), 2));

      end_dis = std::sqrt(std::pow((*closest_it).myposition.x - ((planning.graph_pointer)->point)->x(), 2) +
                          std::pow((*closest_it).myposition.y - ((planning.graph_pointer)->point)->y(), 2) +
                          std::pow((*closest_it).myposition.z - ((planning.graph_pointer)->point)->z(), 2));

      if (close_dis < my_close_dis)
      {
        my_close_dis = close_dis;
        starting.x_ref = ((*closest_it).myposition.x);
        starting.y_ref = ((*closest_it).myposition.y);
        starting.z_ref = ((*closest_it).myposition.z);
      }

      if (end_dis < my_end_dis)
      {
        my_end_dis = end_dis;
        ending.x_ref = ((*closest_it).myposition.x);
        ending.y_ref = ((*closest_it).myposition.y);
        ending.z_ref = ((*closest_it).myposition.z);
      }
    }

    Graph::Point start_pos{starting.x_ref, starting.y_ref, starting.z_ref};
    Graph::Point end_pos{ending.x_ref, ending.y_ref, ending.z_ref};

    // cout << "\nPlanning container has " << planning.node_container.size() << " nodes" << endl;

    Graph mygraph;
    planned.traj = mygraph.fetchPath(start_pos, end_pos, planning.node_container, ufoMAP);
    
    // cout << "\nI am looking for path " << endl;

    if (!planned.traj.empty())
    {

      cout << "\nOriginal path path has " << planned.traj.size() << " nodes " << endl;
    }
    else
    {

      cout << "\nthe planned traj was empty " << endl;
    }
  
  
  }
  else if (!planning.node_container.empty())
  {

    Graph::Point current_state{((planning.node_container.back()).myposition.x), ((planning.node_container.back()).myposition.y), ((planning.node_container.back()).myposition.z)};
    Graph::Point home_state{((planning.node_container.front()).myposition.x), ((planning.node_container.front()).myposition.y), ((planning.node_container.front()).myposition.z)};

    Graph homing_graph;
    planned.traj = homing_graph.fetchPath(current_state, home_state, planning.node_container, ufoMAP);

    if (planned.traj.size() > 1)
    {

      cout << "homing path is valid" << endl;
    }
  }

  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  // cout << "\n\033[;34m Path finding takes... \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;
}


void improvePath()
{

  high_resolution_clock::time_point start_total = high_resolution_clock::now();

  Graph improved_graph;
  Graph::Point finish{((improved.node_container.back()).myposition.x), ((improved.node_container.back()).myposition.y), ((improved.node_container.back()).myposition.z)};
  Graph::Point start{((improved.node_container.front()).myposition.x), ((improved.node_container.front()).myposition.y), ((improved.node_container.front()).myposition.z)};

  improved.traj = improved_graph.fetchPath(start, finish, improved.node_container, ufoMAP);  

  cout << "\nimproved path has " << improved.traj.size() << " Nodes \n" << endl;
  
  high_resolution_clock::time_point stop_total = high_resolution_clock::now();
  auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  // cout << "\n\033[;34m Path improvement takes... \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;
}



void graph_navigation(bool local_nav)
{
  jump_here:
  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  double reiterate = 1;
  double scale_range = 1;

  buildPlanningGraph(false, local_nav, planning.improve, reiterate, scale_range);
  findPath();

  if (planned.traj.size() >= 2)
  {


    final.traj.clear();
    double r_u = 0.1;
    double r_d = 0.1;
    for (auto i = planned.traj.begin(); i != planned.traj.end(); ++i)
    {
      cout << "\n original Z is " << i->z << endl;
    lab:
      ufo::math::Vector3 up((*i).x, (*i).y, (*i).z );
      ufo::math::Vector3 up_plus((*i).x, (*i).y, (*i).z + r_u);
      ufo::math::Vector3 up_minus((*i).x, (*i).y, (*i).z - r_d);
      ufo::geometry::LineSegment line_up(up, up_plus);
      ufo::geometry::LineSegment line_down(up, up_minus);
      if (!isInCollision(ufoMAP, line_up, true, false, true, PLANNING_CHECK_DEPTH))
      {
        r_u = r_u + 0.5;
        goto lab;
      }
      else if (!isInCollision(ufoMAP, line_down, true, false, true, PLANNING_CHECK_DEPTH)) {
        r_d = r_d + 0.5;
        goto lab;
      }
        else
        {
          ufo::math::Vector3 final_up((*i).x, (*i).y, (*i).z + r_u);
          ufo::math::Vector3 final_down((*i).x, (*i).y, (*i).z - r_d);

          ufo::math::Vector3 fin(final_up.x(), final_up.y(), (abs(final_up.z()) - abs(final_down.z())) / 2);

          cout << "\n final Z is " << fin.z() << endl;

          final.traj.push_back({ Graph::Point { fin.x(), fin.y(), fin.z() } });

          r_u = 0.1;
          r_d = 0.1;
        }
    }


    planning.improve = false;
    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    // cout << "\n\033[;34m Graph Based Planning took \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;
    current.temppath_global.poses.clear();
    current.temppath_local.poses.clear();

    std::vector<Graph::planning_node> improvement_nodes;

    improvement_nodes.clear();
    improved.node_container.clear();
    improved.traj.clear();
    
    for (auto insert_helper = final.traj.begin(); insert_helper != final.traj.end(); insert_helper++)
    {
      improvement_nodes.push_back({Graph::Point{insert_helper->x, insert_helper->y, insert_helper->z}});
    }

    for (auto i = improvement_nodes.begin(); i != --improvement_nodes.end(); i++) {

      for (auto i2 = std::next(improvement_nodes.begin(),1); i2 != improvement_nodes.end(); i2++) {

        ufo::math::Vector3 start (i->myposition.x, i->myposition.y, i->myposition.z);
        ufo::math::Vector3 goal (i2->myposition.x, i2->myposition.y, i2->myposition.z);

        ufo::geometry::OBB obb = makeOBB (start, goal , PLANNING_GRAPH_COLISSION_MARGIN);

        if (isInCollision(ufoMAP, obb, false, true, false, PLANNING_CHECK_DEPTH) 
                  && !isInCollision(ufoMAP, obb, true, false, true, PLANNING_CHECK_DEPTH)) {

          improved.node_container.push_back(*i);
          improved.node_container.push_back(*i2);
          (*i).AddConnection(*i2);
          (*i2).AddConnection(*i);            

        }
      }
    }

    improvePath();

    if (improved.traj.size() < 2) {
      goto jump_here;
    }

    for (auto vert_it = improved.traj.begin(); vert_it != improved.traj.end(); vert_it++)
    {
      // cout << " \nX --" << ((*vert_it)).x() << " Y -- " << ((*vert_it)).y() << " Z -- " << ((*vert_it)).z() << endl;

      geometry_msgs::PoseStamped global_nodes;
      global_nodes.pose.position.x = ((*vert_it)).x;
      global_nodes.pose.position.y = ((*vert_it)).y;
      global_nodes.pose.position.z = ((*vert_it)).z;
      global_nodes.pose.orientation.x = 0;
      global_nodes.pose.orientation.y = 0;
      double head_ref;
      if (vert_it != std::prev(improved.traj.end(), 1))
      {
        head_ref = atan2(((*std::next(vert_it, 1))).y - ((*vert_it)).y,
                         ((*std::next(vert_it, 1))).x - ((*vert_it)).x);
      }
      else
      {
        head_ref = atan2( (*std::prev(vert_it,1)).y - (*std::prev(vert_it,2)).y , 
                              (*std::prev(vert_it,1)).x - (*std::prev(vert_it,2)).x );
        // head_ref = view_towards;
      }
      global_nodes.pose.orientation.z = head_ref;
      global_nodes.pose.orientation.w = 1;
      global_nodes.header.stamp = ros::Time::now();
      global_nodes.header.frame_id = ODOM_FRAME_ID;
      
      if (local_nav)
      {
        current.temppath_local.poses.push_back(global_nodes);
        current.temppath_global.poses.clear();
      }
      else
      {
        current.temppath_global.poses.push_back(global_nodes);
        current.temppath_local.poses.clear();
      }
    }

      if (local_nav)
      {
      current.path.type.data = "local";
      current.path.path = current.temppath_local;
      pub.global_path_pub.publish(current.temppath_local);
      }
      else
      {
      current.path.type.data = "global";
      current.path.path = current.temppath_global;
      pub.global_path_pub.publish(current.temppath_global);
      }

    pub.path_to_goal_pub.publish(current.path);
    current.path.path.poses.clear();
    current.path.type.data.clear();

    ufo::math::Vector3 lastpub(std::prev(improved.traj.end(), 1)->x, std::prev(improved.traj.end(), 1)->y,
                               std::prev(improved.traj.end(), 1)->z);
    published.reference = publish_goal(lastpub);
    }
  else
  {
    planning.improve = true;
  label:
    // if (planned.traj.size() < 3 and reiterate < 5)
    if (planned.traj.size() < 2 and reiterate < 6)
    {
      if (switch_global.navigate or home.navigate) {
        
        switch_global.navigate = false;
        // goto jump_here;
        return;
      }
      visualize();
      cout << "it .. : " << reiterate << endl;
      std::cout << "\nCould not find a good path... requesting improvement.... \n";
      reiterate = reiterate + 1;
      scale_range = IMPROVEMENT_SCALE_FACTOR_RANGE + scale_range;
      planning.improve = true;
      buildPlanningGraph(false, local_nav, planning.improve, reiterate, scale_range);
      findPath();
      improvePath();
      goto label;
    }
    else if (planned.traj.size() < 2 and reiterate > 5)
    {
      return;
      start_backtrack.navigate = true;

      nav_msgs::Path retrace_path = backTrack(start_backtrack.navigate);

      for (auto it_rt = (retrace_path.poses.begin()); it_rt != retrace_path.poses.end(); it_rt++)
      {

        cout << "X : " << it_rt->pose.position.x << "\n ";
        cout << "Y : " << it_rt->pose.position.y << "\n ";
        cout << "Z : " << it_rt->pose.position.z << "\n ";
        cout << "--------"
             << "\n ";
        geometry_msgs::PoseStamped global_nodes;
        global_nodes.pose.position.x = it_rt->pose.position.x;
        global_nodes.pose.position.y = it_rt->pose.position.y;
        global_nodes.pose.position.z = it_rt->pose.position.z;
        global_nodes.pose.orientation.x = 0;
        global_nodes.pose.orientation.y = 0;
        global_nodes.pose.orientation.z = atan2( ((std::next(it_rt,1)))->pose.position.y - ((it_rt))->pose.position.y , 
                                                        ((std::next(it_rt,1)))->pose.position.x - ((it_rt))->pose.position.x );
        global_nodes.pose.orientation.w = 1;
        global_nodes.header.stamp = ros::Time::now();
        global_nodes.header.frame_id = ODOM_FRAME_ID;

        current.temppath_global.poses.push_back(global_nodes);
        current.path.path = current.temppath_global;
        if (local_nav)
        {
          current.path.type.data = "local";
        }
        else
        {
          current.path.type.data = "global";
          pub.global_path_pub.publish(current.temppath_global);
        }
      }

      pub.path_to_goal_pub.publish(current.path);
      
      std::vector<geometry_msgs::PoseStamped>::const_iterator it_r = std::prev(current.temppath_global.poses.end(), 1);
      ufo::math::Vector3 lastpub(it_r->pose.position.x, it_r->pose.position.y, it_r->pose.position.z);
      published.reference = publish_goal(lastpub);

      current.path.path.poses.clear();
      current.path.type.data.clear();
      current.temppath_global.poses.clear();
    }
    
  }
  planned.traj.clear();
}

void return_to_base_planning()
{
  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  double reiterate = 1;
  double scale_range = 1;
  buildPlanningGraph(true, false, planning.improve, reiterate, scale_range);
  findPath();

  if (!planned.traj.empty())
  {

    planning.improve = false;
    high_resolution_clock::time_point stop_total = high_resolution_clock::now();
    auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
    cout << "\n\033[;34mReturn to base planning took \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;

      for (auto vert_it = std::next(planned.traj.begin(), 0); vert_it != std::prev(planned.traj.end(), 0); vert_it++)
      {

        geometry_msgs::PoseStamped global_nodes;
        global_nodes.pose.position.x = ((*vert_it)).x;
        global_nodes.pose.position.y = ((*vert_it)).y;
        global_nodes.pose.position.z = ((*vert_it)).z;
        global_nodes.header.stamp = ros::Time::now();
        global_nodes.header.frame_id = ODOM_FRAME_ID;
        global_nodes.pose.orientation.x = 0;
        global_nodes.pose.orientation.y = 0;
        global_nodes.pose.orientation.z = atan2( ((*std::next(vert_it,1))).y - ((*vert_it)).y , 
                                                        ((*std::next(vert_it,1))).x - ((*vert_it)).x );
        global_nodes.pose.orientation.w = 1;
        current.temppath_homing.poses.push_back(global_nodes);
        current.path.path = current.temppath_homing;
        current.path.type.data = "homing";
      }

    pub.path_to_goal_pub.publish(current.path);
    pub.homing_path_pub.publish(current.temppath_homing);
    current.path.path.poses.clear();
    current.path.type.data.clear();
    current.temppath_homing.poses.clear();
    ufo::math::Vector3 lastpub(planned.traj.back().x, planned.traj.back().y, planned.traj.back().z);
    published.reference = publish_goal(lastpub);
  }

  planned.traj.clear();
}

//----------------------------------------- MAIN ---------------------------------------------------//

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ACL_EXPLORATION");
  ros::NodeHandle nh;

  ros::param::get("/ODOM_FRAME_ID_", ODOM_FRAME_ID);
  ros::param::get("/INITIALIZE_", INITIALIZE);
  ros::param::get("/RESOLUTION_", RESOLUTION);
  ros::param::get("/RADIOUS_", RADIOUS);
  ros::param::get("/NEXT_PUB_DISTANCE_", NEXT_PUB_DISTANCE);
  ros::param::get("/LOCAL_RANGE_", LOCAL_RANGE);
  ros::param::get("/RAW_GRAPH_COLISSION_MARGIN_", RAW_GRAPH_COLISSION_MARGIN);
  ros::param::get("/MIN_DIST_NODE_", MIN_DIST_NODE);
  ros::param::get("/PLANNING_GRAPH_COLISSION_MARGIN_", PLANNING_GRAPH_COLISSION_MARGIN);
  ros::param::get("/RAW_FRONTIERS_COLISSION_MARGIN_", RAW_FRONTIERS_COLISSION_MARGIN);
  ros::param::get("/PLANNING_CHECK_DEPTH_", PLANNING_CHECK_DEPTH);
  ros::param::get("/MIN_ITERATION_LOCAL_", MIN_ITERATION_LOCAL);
  ros::param::get("/MIN_ITERATION_GLOBAL_", MIN_ITERATION_GLOBAL);
  ros::param::get("/MIN_PLANNING_SPACE_LOCAL_", MIN_PLANNING_SPACE_LOCAL);
  ros::param::get("/MIN_PLANNING_SPACE_GLOBAL_", MIN_PLANNING_SPACE_GLOBAL);
  ros::param::get("/MAX_ITERATION_LOCAL_", MAX_ITERATION_LOCAL);
  ros::param::get("/MAX_ITERATION_GLOBAL_", MAX_ITERATION_GLOBAL);
  ros::param::get("/MAX_PLANNING_SPACE_LOCAL_", MAX_PLANNING_SPACE_LOCAL);
  ros::param::get("/MAX_PLANNING_SPACE_GLOBAL_", MAX_PLANNING_SPACE_GLOBAL);
  ros::param::get("/IMPROVEMENT_SCALE_FACTOR_IT_", IMPROVEMENT_SCALE_FACTOR_IT);
  ros::param::get("/IMPROVEMENT_SCALE_FACTOR_RANGE_", IMPROVEMENT_SCALE_FACTOR_RANGE);
  ros::param::get("/ODOMETRY_IN_", ODOMETRY_IN);
  ros::param::get("/UFOMAP_", MAP_TOPIC);
  ros::param::get("/REF_TOPIC_", REF_TOPIC);
  ros::param::get("/PATH_TOPIC_", PATH_TOPIC);
  ros::param::get("/FIRST_REF_X_", FIRST_REF_X);
  ros::param::get("/FIRST_REF_Y_", FIRST_REF_Y);
  ros::param::get("/FIRST_REF_Z_", FIRST_REF_Z);

  // dynamic_reconfigure::Server<acl_exploration::ServerConfig> server;
  // dynamic_reconfigure::Server<acl_exploration::ServerConfig>::CallbackType f;
  // f = boost::bind(&config_callback, _1, _2);
  // server.setCallback(f);
  pub.points_pub = nh.advertise<visualization_msgs::Marker>("/planning_graph", 1);
  pub.robot_box_vis_pub = nh.advertise<visualization_msgs::Marker>("/ROBOT_BOX", 1);
  pub.current_goal_vis_pub = nh.advertise<visualization_msgs::Marker>("/current_goal", 1);
  pub.current_unknowns_visualization_pub = nh.advertise<visualization_msgs::Marker>("/current_unknowns", 1);
  pub.alc_points_visualization_pub = nh.advertise<visualization_msgs::Marker>("/ALC_POINTS", 1);
  pub.alc_path_visualization_pub = nh.advertise<visualization_msgs::Marker>("/ALC_PATH", 1);
  pub.best_frontier_pub = nh.advertise<geometry_msgs::PoseStamped>("REF_TOPIC_", 1);
  pub.path_to_goal_pub = nh.advertise<acl_exploration::path>("PATH_TOPIC_", 1);
  pub.local_path_pub = nh.advertise<nav_msgs::Path>("/local_path", 1);
  pub.global_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 1);
  pub.homing_path_pub = nh.advertise<nav_msgs::Path>("/homing_path", 1);
  pub.explored_path = nh.advertise<nav_msgs::Path>("/explored_path", 1);
  pub.hits_pub = nh.advertise<visualization_msgs::Marker>("/Traversable_frontiers", 1);
  pub.local_nodes_pub = nh.advertise<visualization_msgs::Marker>("/local_frontier_nodes_vis", 1);
  pub.global_nodes_pub = nh.advertise<visualization_msgs::Marker>("/global_frontier_nodes_vis", 1);
  pub.total_execution_pub = nh.advertise<std_msgs::Int64>("/total_execution_time", 1);
  pub.local_execution_pub = nh.advertise<std_msgs::Int64>("/local_execution_time", 1);
  pub.global_execution_pub = nh.advertise<std_msgs::Int64>("/global_execution_time", 1);
  // pub.map_pub = nh.advertise<ufomap_msgs::UFOMapStamped>("/acl_full_map", 11);
  ros::Subscriber map_sub = nh.subscribe("UFOMAP_", 1, mapCallback);
  ros::Subscriber sub = nh.subscribe("ODOMETRY_IN_", 1, odomCallback);
  ros::Subscriber front_sub = nh.subscribe("/frontiers_nodes", 1, frontiersCallback);
  ros::Subscriber homing_sub = nh.subscribe("/return_home", 1, homingCallback);
  ros::Subscriber explore_sub = nh.subscribe("/explore", 1, exploreCallback);
  ros::Subscriber assist_sub = nh.subscribe("/go_global", 1, switchCallback);
  ros::Rate rate(10);

  position_received = true;
  published.reference.header.stamp = ros::Time::now();
  published.reference.header.frame_id = ODOM_FRAME_ID;
  current.path.path.header.stamp = ros::Time::now();
  current.path.path.header.frame_id = ODOM_FRAME_ID;

  current.temppath_global.header.stamp = ros::Time::now();
  current.temppath_global.header.frame_id = ODOM_FRAME_ID;

  current.temppath_local.header.stamp = ros::Time::now();
  current.temppath_local.header.frame_id = ODOM_FRAME_ID;

  current.temppath_homing.header.stamp = ros::Time::now();
  current.temppath_homing.header.frame_id = ODOM_FRAME_ID;

  home.navigate = false;
  explore.navigate = false;
  switch_global.navigate = false;

  std::list<int> exec;
  ofstream myfile;
  // myfile.open("/home/aakapatel/Documents/example.txt");

  int it = 0;
  while (ros::ok())
  {

    high_resolution_clock::time_point start_total = high_resolution_clock::now();

    /*
      main computations to be executed here on
    */

    if (firstTime and explore.navigate and INITIALIZE)
    {

      firstTime = false;
      geometry_msgs::PoseStamped first_ref;
      first_ref.pose.position.x = FIRST_REF_X;
      // first_ref.pose.position.y = current.position.y();
      first_ref.pose.position.y = FIRST_REF_Y;
      first_ref.pose.position.z = FIRST_REF_Z;
      first_ref.pose.orientation.w = 0;
      first_ref.pose.orientation.x = 0;
      first_ref.pose.orientation.y = 0;
      first_ref.pose.orientation.z = 0;
      pub.best_frontier_pub.publish(first_ref);
      published.reference = first_ref;
    }

    if ((next_pub() or switch_global.navigate or home.navigate) and !finish.navigate)
    {
      // fetchUnknownNodes(current.position, ufoMAP);

      // lookAround();

      if (local.navigate and explore.navigate and !home.navigate and !switch_global.navigate)
      {

        high_resolution_clock::time_point start_total = high_resolution_clock::now();

        exploreLocally();

        graph_navigation(true);

        high_resolution_clock::time_point stop_total = high_resolution_clock::now();
        auto local_planning_duration = duration_cast<microseconds>(stop_total - start_total);

        std_msgs::Int64 local_planning_time;

        local_planning_time.data = int(local_planning_duration.count());

        pub.local_execution_pub.publish(local_planning_time);

        exec.push_back(local_planning_duration.count());
      }
      else if ((!local.navigate and global.navigate and explore.navigate and !home.navigate) or switch_global.navigate)
      {

        high_resolution_clock::time_point start_total = high_resolution_clock::now();

        graph_navigation(false);

        high_resolution_clock::time_point stop_total = high_resolution_clock::now();
        auto global_planning_duration = duration_cast<std::chrono::milliseconds>(stop_total - start_total);

        std_msgs::Int64 global_planning_time;
        global_planning_time.data = int(global_planning_duration.count());
        pub.global_execution_pub.publish(global_planning_time);

        switch_global.navigate = false;
      }

      if (home.navigate)
      {
        std::cout << "returning home" << std::endl;
        local.navigate = false;
        global.navigate = false;
        // explore.navigate = false;

        return_to_base_planning();
        finish.navigate = true;
      }

      

      high_resolution_clock::time_point stop_total = high_resolution_clock::now();
      auto duration_total = duration_cast<std::chrono::milliseconds>(stop_total - start_total);
      cout << "\n \033[;32mTotal planning execution time: \033[0m " << duration_total.count() << " \033[;34m ms \033[0m\n " << endl;

      // std_msgs::Int64 exec_time;

      // exec_time.data = int(duration_total.count());

      // pub.total_execution_pub.publish(exec_time);

      // exec.push_back(duration_total.count());
    }
    
    visualize();
    ros::spinOnce();
    rate.sleep();
  }

  /*

      myfile.open("/home/aakapatel/Documents/example.txt");
      if (myfile.is_open())
      {
        cout << "file was opened " << endl;
        for (auto i = exec.begin(); i != exec.end(); i++) {
          myfile << "execution_time " <<  *i << " iteration " << std::distance(exec.begin(), i) << endl;
        }
        myfile.close();
      }
      else { cout << "Unable to open file";
      }

  */

  return 0;
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- //
// TODO LIST

/*
1) Fix frontiers detection -- done
2) Compute information gain -- TBD
3) Fix heading deviation based search -- done
4) Global graph based strategy -- done
5) Active loop closing -- candidates can be computed from the graph
6) Loop closing candidates to be included as part of hueristic function
7) Compute global path cost based on hueristic and shortest path
8) Improve computation time
9) Implement adaptive graph expansion
10) FIX Plots
-- AP
*/
