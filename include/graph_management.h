#ifndef GRAPH_MANAGEMENT_H
#define GRAPH_MANAGEMENT_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
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
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
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
#include "stage_planner/frontierRequest.h"
#include <omp.h>

class GraphManager {

public:
 
    class node {    
    
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

    class Graph
    {

    public:
        struct Point
        {
        Point(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z){};
        bool operator==(const Point &o) const { return o.x == x && o.y == y && o.z == z; }
        double DistTo(Point &other) const { return (std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2) + std::pow(other.z - z, 2))); };
        double Dist_HeuristicTo(Point &other) const { return (0.1 * (std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2) + std::pow(other.z - z, 2)))); };
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
        bool operator==(const vertex &o) const { return (o.location.x == location.x) && (o.location.y == location.y) && (o.location.z == location.z); }
        bool operator<(const vertex &o) const { return (total_dist < o.total_dist); }

        void createEdge(vertex subnode)
        {
          edges->push_back(subnode);
        }
        int getEdges () {
          return edges->size();
        }

        };

      std::vector<Point> fetchPath(Point &start_pos, Point &end_pos, std::vector<vertex> all_vertices);
    };
    
   
    GraphManager(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
        
    virtual void updateCurrentPosition(const nav_msgs::Odometry::ConstPtr& pos);
    
    virtual void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &map);
    
    double calculateDistance(ufo::math::Vector3 point1, ufo::math::Vector3 point2);
    
    bool isInCollision(ufo::map::OccupancyMapColor const &map,
           ufo::geometry::BoundingVar const &bounding_volume,
           bool occupied_space, bool free_space,
           bool unknown_space, ufo::map::DepthType min_depth);
    
    ufo::geometry::OBB makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius); 
    
    double calculateAngle(ufo::math::Vector3 fpoint, ufo::math::Vector3 position, double heading);
    
    bool isLocal(ufo::math::Vector3 point);
    
    ufo::math::Vector3 fetchCurrentPosition ();
    
    double computeVolumetricGain (ufo::math::Vector3 frontier, ufo::map::OccupancyMapColor const &map);
    
    ufo::math::Vector3 tunePlanningSpace(float range, ufo::math::Vector3 root_node, 
                                    ufo::map::OccupancyMapColor const &map);

    std::vector<GraphManager::node *> extractNodes(bool require_local, bool build_reserved_tree, ufo::math::Vector3 root_node,
                                        ufo::math::Vector3 target, int max_iterations, 
                                            float range, ufo::map::OccupancyMapColor const &map);

    ufo::math::Vector3 pointToUfo (geometry_msgs::Point p); 

    std::vector<ufo::math::Vector3> pointsToUfo (std::vector<geometry_msgs::Point> points);

    void fetchExplorationTargets ();

    void constructLocalGraph();


    //variables
    bool firstTime = true;
    ufo::math::Vector3 m_currentPosition;
    double m_currentHeading;
    double m_local_space;
    double m_local_fov;
    bool map_received;
    std::string m_ufomap;
    bool m_max_allowed_Z;
    double m_obb_radius;
    int m_planning_depth;
    double m_volumetric_gain_threashold;
    bool check = false;
    float SCALER_X, SCALER_Y, SCALER_Z, highest_x, highest_y ,highest_z, lowest_x, lowest_y, lowest_z;
    std::vector<node *> reserved_nodes;
    double m_min_dist_bw_nodes;
    ufo::math::Vector3 m_local_exploration_target;
    std::vector<ufo::math::Vector3> m_global_repos_targets;
    int m_max_iterations_local;
    double m_max_edge_length;
    


    ros::NodeHandle m_nh;
    std::string m_worldFrameId; // the map frame
    std::string m_odometry;
    std::string m_baseFrameId; // base of the robot for ground plane filtering
    nav_msgs::Odometry m_currentPose;
    nav_msgs::Odometry m_prevPose; //Holds the point where the direction changed.
    ros::ServiceClient m_frontier_client;



    //Subscriber
    message_filters::Subscriber<nav_msgs::Odometry>* m_poseSubscriber;
    message_filters::Subscriber<ufomap_msgs::UFOMapStamped>* m_ufoMapSubscriber;


    //Publisher
    ros::Publisher m_frontierPub;
    ros::Publisher m_frontierSelectionPub;
    ros::Publisher command_pub;

};


#endif
