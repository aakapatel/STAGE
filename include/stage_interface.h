#ifndef STAGE_INTERFACE_H
#define STAGE_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <std_msgs/ColorRGBA.h>
#include "stage_planner/Frontiers.h"
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
#include <dynamic_reconfigure/server.h>
#include "stage_planner/path.h"
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


class StageInterface {
public:
    StageInterface(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
    ros::NodeHandle m_nh;
    message_filters::Subscriber<nav_msgs::Odometry>* m_poseSubscriber;
    std::string m_odometry;
    virtual void odomCallback(const nav_msgs::Odometry::ConstPtr& pos);
    nav_msgs::Odometry m_currentPose;

};




#endif
