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

class GraphManager {
    public:
        
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

        //variables
        bool firstTime = true;
        ufo::math::Vector3 m_currentPosition;
        double m_currentHeading;
        double LOCAL_SPACE;
        double LOCAL_FOV;
        bool map_received;
        std::string m_ufomap;
        bool m_max_allowed_Z;
        double OBB_RADIUS;
        int PLANNING_CHECK_DEPTH;
        double VOLUMETRIC_GAIN_THREASHOLD;
        bool check = false;



        ros::NodeHandle m_nh;
        std::string m_worldFrameId; // the map frame
        std::string m_odometry;
        std::string m_baseFrameId; // base of the robot for ground plane filtering
        nav_msgs::Odometry m_currentPose;
        nav_msgs::Odometry m_prevPose; //Holds the point where the direction changed.

        //Subscriber
        message_filters::Subscriber<nav_msgs::Odometry>* m_poseSubscriber;
        message_filters::Subscriber<ufomap_msgs::UFOMapStamped>* m_ufoMapSubscriber;


        //Publisher
        ros::Publisher m_frontierPub;
        ros::Publisher m_frontierSelectionPub;
        ros::Publisher command_pub;

};


#endif
