#ifndef GRAPH_MANAGEMENT_H
#define GRAPH_MANAGEMENT_H

#include "frontiers_extraction.h"
#include <omp.h>

class GraphManager
{

public:
  GraphManager(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));

  class node
  {

  public:
    ufo::math::Vector3 *point;
    node *myParent = nullptr;
    double distanceToParent = -1;
    std::vector<node *> myChilds{};
    std::vector<node *> myPath{};
    bool isTraversableByDrone = true;
    double distanceToGoal;
    node(float x, float y, float z)
    {
      point = new ufo::math::Vector3(x, y, z);
    }
    node(ufo::math::Vector3 *givenPoint)
    {
      point = new ufo::math::Vector3(givenPoint->x(), givenPoint->y(), givenPoint->z());
    }
    ufo::math::Vector3 *getNodePointer();
    void addChild(node *newChild);
    void addParent(node *newParent);
    void setTraversabilityStatus(bool Traversable);
    bool getTraversabilityStatus();
    void readyForDeletion();
  };

  class PoseGraph
  {
  public:
    struct pg_node
    {
      geometry_msgs::PoseWithCovariance pose_;
      float associated_sensor_measurement_;
      void add_sensor_measurement(float sensor_measurement)
      {
        associated_sensor_measurement_ = sensor_measurement;
      }
      pg_node(geometry_msgs::PoseWithCovariance my_pose)
      {
        pose_ = my_pose;
      }
    };

    std::vector<pg_node> pg_nodes_;

    void add_pg_node(pg_node N)
    {
      pg_nodes_.push_back(N);
    }

    std::shared_ptr<std::vector<pg_node>> pg_edges_;

    void connectNodes(pg_node &previous_node, pg_node &new_node)
    {
      pg_edges_->push_back(previous_node);
      pg_edges_->push_back(new_node);
    }
    
    struct planning_instance
    {
      bool retracing;
      int planning_iteration;
      std::vector<node *> reserved_samples;
      planning_instance(int pi_) 
      {
        // retracing = retracing_;
        planning_iteration = pi_;
        // reserved_samples = rs;
      }
    };
    
    std::vector<planning_instance> planning_instances;
   
  };
  
  PoseGraph poseGraph;
  int planning_iteration_ = 0;

  class Graph
  {

  public:
    struct Point
    {
      Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z){};
      bool operator==(const Point &o) const { return o.x == x && o.y == y && o.z == z; }
      double DistTo(Point &other) const { return (std::sqrt(std::pow(other.x - x, 2) +
                                                            std::pow(other.y - y, 2) + std::pow(other.z - z, 2))); };
      double Dist_HeuristicTo(Point &other) const { return (0.1 * (std::sqrt(std::pow(other.x - x, 2) +
                                                                             std::pow(other.y - y, 2) + std::pow(other.z - z, 2)))); };
      double x, y, z;
    };
    struct vertex
    {

      vertex(Point location = Point(), vertex *parent = nullptr) : location(location), parent(parent){};
      Point location;

      double dist_to_start = 0;
      double total_dist = INT_MAX;

      std::shared_ptr<vertex> parent;
      std::shared_ptr<std::vector<vertex>> edges{new std::vector<vertex>};
      bool operator==(const vertex &o) const { return (o.location.x == location.x) &&
                                                      (o.location.y == location.y) && (o.location.z == location.z); }
      bool operator<(const vertex &o) const { return (total_dist < o.total_dist); }

      void createEdge(vertex subnode)
      {
        edges->push_back(subnode);
      }
      int getEdges()
      {
        return edges->size();
      }
    };

    std::vector<Point> fetchPath(Point &start_pos, Point &end_pos, std::vector<vertex> all_vertices);
  };

  virtual void updateCurrentPosition(const nav_msgs::Odometry::ConstPtr &pos);

  virtual void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &map);

  double calculateDistance(ufo::math::Vector3 point1, ufo::math::Vector3 point2);

  bool isInCollision(ufo::map::OccupancyMapColor const &map,
                     ufo::geometry::BoundingVar const &bounding_volume,
                     bool occupied_space, bool free_space,
                     bool unknown_space, ufo::map::DepthType min_depth);

  ufo::geometry::OBB makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius);

  double calculateAngle(ufo::math::Vector3 fpoint, ufo::math::Vector3 position, double heading);

  bool isLocal(ufo::math::Vector3 point);

  ufo::math::Vector3 fetchCurrentPosition();

  double computeVolumetricGain(ufo::math::Vector3 frontier, ufo::map::OccupancyMapColor const &map);

  ufo::math::Vector3 tunePlanningSpace(float range, ufo::math::Vector3 root_node,
                                       ufo::map::OccupancyMapColor const &map);

  std::vector<node *> sampleNodes(ufo::math::Vector3 root_node, int max_iterations,
                                  float range, ufo::map::OccupancyMapColor const &map);

  ufo::math::Vector3 pointToUfo(geometry_msgs::Point p);

  std::vector<ufo::math::Vector3> pointsToUfo(std::vector<geometry_msgs::Point> points);

  void fetchExplorationTargets();

  bool isLastTargetFrontier();

  std::vector<Graph::vertex> constructGraph(int max_itr, float planning_range);

  std::vector<Graph::Point> planPath();
  
  nav_msgs::Path fetchExplorationPath();

  void constructGlobalGraph ();

  void planHomingPath ();

  
  bool nextExplorationPath (stage_planner::explorationPath::Request &req,
                             stage_planner::explorationPath::Response &res);
  
  Eigen::Quaternion<float> Euler2Quaternion(ufo::math::Vector3 v);

private:
  // variables
  bool firstTime = true;
  ufo::math::Vector3 m_currentPosition;
  double m_currentHeading;
  double m_local_space, m_global_space;
  double m_local_fov;
  bool map_received;
  bool initialize_global_graph = true;
  std::string m_ufomap;
  bool m_max_allowed_Z;
  double m_obb_radius;
  int m_planning_depth;
  double m_volumetric_gain_threashold;
  bool check = false;
  float SCALER_X, SCALER_Y, SCALER_Z, highest_x, highest_y, highest_z, lowest_x, lowest_y, lowest_z;
  std::vector<node *> reserved_nodes;
  double m_min_dist_bw_nodes;
  ufo::math::Vector3 m_local_exploration_target;
  std::vector<ufo::math::Vector3> m_global_repos_targets;
  int m_iterations_local;
  int m_iterations_global;
  double m_max_edge_length;
  bool m_reserved_graph;
  bool local_exploration;
  bool global_exploration;
  bool run_global_planner = false;
  ufo::math::Vector3 exploration_target;
  std::vector<GraphManager::Graph::vertex> global_graph_container_;
  nav_msgs::Path homing_path_;
  ufo::math::Vector3 home_position;
  // std::unordered_map<Graph::vertex, Graph::vertex> graph_;
  int vertex_id_ = 0;
  std::vector<float> vert_vec;
  std::unordered_map<int, GraphManager::Graph::vertex> graph_;
  
   std::vector<Graph::Point> Astar_path; 


  ros::NodeHandle m_nh;
  std::string m_worldFrameId; // the map frame
  std::string m_odometry;
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  nav_msgs::Odometry m_currentPose;
  nav_msgs::Odometry m_prevPose; // Holds the point where the direction changed.
  ros::ServiceClient m_frontier_client;
  ros::ServiceClient m_target_query_client;
  ros::ServiceServer m_exploration_path_srv;
  
    
  std::vector<ufo::math::Vector3> visited_exploration_targets;
  
    
  // Subscriber
  message_filters::Subscriber<nav_msgs::Odometry> *m_poseSubscriber;
  message_filters::Subscriber<ufomap_msgs::UFOMapStamped> *m_ufoMapSubscriber;

  // Publisher
  ros::Publisher m_frontierPub;
  ros::Publisher m_frontierSelectionPub;
  ros::Publisher command_pub;
};

#endif
