#ifndef FRONTIERS_EXTRACTION_H
#define FRONTIERS_EXTRACTION_H

#include <ros/ros.h>
#include <bits/stdc++.h>
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
#include "stage_planner/isFrontier.h"
#include "stage_planner/explorationPath.h"

class FrontierExtraction
{
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
    typedef pcl::PointXYZ PCLPoint;
    typedef octomap::point3d point3d;
    typedef octomap::OcTree OcTreeT;
    typedef octomap::KeySet KeySet;

    FrontierExtraction(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));

    virtual void updateCurrentPosition(const nav_msgs::Odometry::ConstPtr &pos);

    virtual void insertPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud);

    virtual void insertPointCloudScan(const tf::Point &sensorOrigin, const PCLPointCloud &ground,
                                      const PCLPointCloud &nonground);

    virtual void extractFrontiers(const tf::Point &sensorOrigin, const ros::Time &rostime = ros::Time::now());

    virtual void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &map);

    bool verifyFrontier(float x, float y, float z, point3d sensorOrigin);

    double calculateDistance(ufo::math::Vector3 point1, ufo::math::Vector3 point2);

    bool isInCollision(ufo::map::OccupancyMapColor const &map,
                       ufo::geometry::BoundingVar const &bounding_volume,
                       bool occupied_space, bool free_space,
                       bool unknown_space, ufo::map::DepthType min_depth);

    ufo::geometry::OBB makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius);

    double calculateAngle(ufo::math::Vector3 fpoint, ufo::math::Vector3 position, double heading);

    bool isLocal(ufo::math::Vector3 point);

    ufo::math::Vector3 fetchCurrentPosition();

    std::vector<ufo::math::Vector3> fetchFrontiers();

    std::vector<ufo::math::Vector3> fetchLocalFrontiers();

    std::vector<ufo::math::Vector3> fetchGlobalFrontiers();

    std::vector<ufo::math::Vector3> fetchLocallyTraversableFrontiers();

    double computeVolumetricGain(ufo::math::Vector3 frontier, ufo::map::OccupancyMapColor const &map);

    ufo::math::Vector3 fetchInformativeFrontier();

    std::vector<geometry_msgs::Point> fetchGlobalRepositioningTargets();

    bool exportFrontierTargets(stage_planner::frontierRequest::Request &req,
                               stage_planner::frontierRequest::Response &res);
    
    bool queryFrontierTarget(stage_planner::isFrontier::Request &req,
                             stage_planner::isFrontier::Response &res);

    geometry_msgs::Point ufoToPoint(ufo::math::Vector3 v);

private:
    // variables
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
    ros::ServiceServer m_frontier_srv;
    ros::ServiceServer m_target_query_srv;
    tf::TransformListener m_tfListener;
    std::string m_worldFrameId; // the map frame
    std::string m_odometry;
    std::string cloud_in;
    std::string m_baseFrameId; // base of the robot for ground plane filtering
    OcTreeT *m_octree;
    OcTreeT *o_octree;
    octomap::KeyRay m_keyRay; // temp storage for ray casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;
    octomap::point3d m_last_point;
    octomap::KeySet finalFrontiers;
    // Octree
    double m_res;
    double m_treeDepth;
    int m_maxTreeDepth;
    nav_msgs::Odometry m_currentPose;
    nav_msgs::Odometry m_prevPose; // Holds the point where the direction changed.
    double m_occupancyMinZ;
    double m_occupancyMaxZ;

    double m_maxRange;
    double m_colorFactor;
    std_msgs::ColorRGBA m_color;
    std_msgs::ColorRGBA m_colorFree;

    bool m_useHeightMap;
    //  Filtering constants
    bool m_filterGroundPlane;
    double m_pointcloudMinX;
    double m_pointcloudMaxX;
    double m_pointcloudMinY;
    double m_pointcloudMaxY;
    double m_pointcloudMinZ;
    double m_pointcloudMaxZ;
    double m_distance_deletion;
    int numberOfUnknowns;
    int j = 0;
    const float node_discovered = 1;
    const float node_frontier = 2;
    octomap::point3d_list frontierPointList;
    bool firstTime = true;
    bool xFiltering;
    bool yFiltering;
    bool zFiltering;
    int totalVoxels = 0;

    std::list<geometry_msgs::Point> frontiersVisual;
    // Subscriber
    message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSubscriber;
    tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSubscriber;
    message_filters::Subscriber<nav_msgs::Odometry> *m_poseSubscriber;
    message_filters::Subscriber<ufomap_msgs::UFOMapStamped> *m_ufoMapSubscriber;

    // Publisher
    ros::Publisher m_frontierPub;
    ros::Publisher m_frontierSelectionPub;
    ros::Publisher command_pub;

protected:
    inline static void updateMinKey(const octomap::OcTreeKey &in, octomap::OcTreeKey &min)
    {
        for (unsigned i = 0; i < 3; ++i)
            min[i] = std::min(in[i], min[i]);
    };

    inline static void updateMaxKey(const octomap::OcTreeKey &in, octomap::OcTreeKey &max)
    {
        for (unsigned i = 0; i > 3; ++i)
            max[i] = std::max(in[i], max[i]);
    };
    /// Test if key is within update area of map (2D, ignores height)
    inline bool isInUpdateBBX(const OcTreeT::iterator &it) const
    {
        // 2^(tree_depth-depth) voxels wide:
        unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
        octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
        return (key[0] + voxelWidth >= m_updateBBXMin[0] && key[1] + voxelWidth >= m_updateBBXMin[1] && key[0] <= m_updateBBXMax[0] && key[1] <= m_updateBBXMax[1]);
    }
};

#endif
