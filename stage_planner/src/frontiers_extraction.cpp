#include "frontiers_extraction.h"

using namespace octomap;
using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

FrontierExtraction::FrontierExtraction(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                                       MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel> *map_manager)
    : nh_(nh),
      nh_private_(nh_private),
      map_manager_(map_manager),
      m_maxRange(10.0),
      m_filterGroundPlane(false),
      m_pointcloudMinX(-std::numeric_limits<double>::max()),
      m_pointcloudMaxX(std::numeric_limits<double>::max()),
      m_pointcloudMinY(-std::numeric_limits<double>::max()),
      m_pointcloudMaxY(std::numeric_limits<double>::max()),
      m_pointcloudMinZ(-std::numeric_limits<double>::max()),
      m_pointcloudMaxZ(std::numeric_limits<double>::max()),
      m_occupancyMinZ(-std::numeric_limits<double>::max()),
      m_occupancyMaxZ(std::numeric_limits<double>::max()),
      numberOfUnknowns(18),
      m_worldFrameId("world"),
      m_odometry("/hummingbird/ground_truth/odometry"),
      cloud_in("/hummingbird/velodyne_points"),
      m_res(0.5),
      m_treeDepth(0),
      m_maxTreeDepth(0),
      xFiltering(true),
      yFiltering(true),
      zFiltering(true),
      m_distance_deletion(8.0),
      LOCAL_SPACE(10),
      LOCAL_FOV(40),
      m_ufomap("/ufomap_mapping_server_node/map"),
      m_voxblox_topic("/graph_management_node/occupied_nodes"),
      max_allowed_height_(10),
      OBB_RADIUS(0.5),
      mapDepth_(0),
      VOLUMETRIC_GAIN_THREASHOLD(10),
      robot_type_("kAerial")

{
    double probHit, probMiss, thresMin, thresMax;
    // Set variables
    nh_private_.param("frame_id", m_worldFrameId, m_worldFrameId);
    nh_private_.param("octree_resolution", m_res, m_res);
    nh_private_.param("sensor_model/hit", probHit, 0.7);
    nh_private_.param("sensor_model/miss", probMiss, 0.4);
    nh_private_.param("sensor_model/min", thresMin, 0.12);
    nh_private_.param("sensor_model/max", thresMax, 0.97);
    nh_private_.param("sensor_model/max_range", m_maxRange, m_maxRange);
    nh_private_.param("point_cloud_topic", cloud_in, cloud_in);
    nh_private_.param("frontier_clearing_range", m_distance_deletion, m_distance_deletion);
    nh_private_.param("odometry_topic", m_odometry, m_odometry);
    nh_private_.param("unknowns_threashold", numberOfUnknowns, numberOfUnknowns);
    nh_private_.param("local_AABB_size", LOCAL_SPACE, LOCAL_SPACE);
    nh_private_.param("local_FOV", LOCAL_FOV, LOCAL_SPACE);
    nh_private_.param("truncate_frontiers_Z", max_allowed_height_, max_allowed_height_);
    nh_private_.param("robot_type", robot_type_, robot_type_);
    
    nh_private_.param("safety_margin", safety_margin, safety_margin);
    nh_private_.param("robot_size", robot_box_size, robot_box_size);

    
    // Setting up octree
    m_octree = new OcTreeT(m_res);
    m_octree->setProbHit(probHit);
    m_octree->setProbMiss(probMiss);
    m_octree->setClampingThresMin(thresMin);
    m_octree->setClampingThresMax(thresMax);
    m_octree->enableChangeDetection(true);
    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;


    // Subscribers

    m_pointCloudSubscriber = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, cloud_in, 5);

    m_tfPointCloudSubscriber = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSubscriber, m_tfListener, m_worldFrameId, 5);
    m_tfPointCloudSubscriber->registerCallback(boost::bind(&FrontierExtraction::insertPointCloudCallback, this, _1));

    m_poseSubscriber = nh_.subscribe(m_odometry, 10, &FrontierExtraction::odomCallback, this);



    // Publishers

    m_frontierPub = nh_.advertise<visualization_msgs::MarkerArray>("/frontier_nodes_vis", 10);

    frontiersVisPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/frontiers", 10);

    m_volume_pub = nh_.advertise<std_msgs::Int64>("/stage_explored_volume", 1);
    
    // Services

    m_frontier_srv = nh_.advertiseService("request_frontier_target", &FrontierExtraction::exportFrontierTargets, this);

    m_target_query_srv = nh_.advertiseService("query_frontier_target", &FrontierExtraction::queryFrontierTarget, this);

    robot_box_size_ << robot_box_size[0], robot_box_size[1], robot_box_size[2];
    safety_margin_ << safety_margin[0], safety_margin[1], safety_margin[2];
    
    map_manager_->setRobotRadius(robot_box_size_.norm() / 2.0);
    
    if (robot_type_ == "kLegged")
    {
        max_allowed_height_ = 1;
    }
};



// Insert LiDAR point cloud to generate octree

void FrontierExtraction::insertPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    PCLPointCloud pc;
    pcl::fromROSMsg(*cloud, pc);

    tf::StampedTransform sensorToWorldTf;
    try
    {
        m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    /***********************************
     ******** Filtering ****************
     **********************************/

    pcl::PassThrough<PCLPoint> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);

    pcl::PassThrough<PCLPoint> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);

    pcl::PassThrough<PCLPoint> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    PCLPointCloud pc_ground;
    PCLPointCloud pc_nonground;

    pcl::transformPointCloud(pc, pc, sensorToWorld);
    // just filter height range:
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    pc_nonground = pc;
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;

    insertPointCloudScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);
    extractFrontiers(sensorToWorldTf.getOrigin(), cloud->header.stamp);
}

// This function distingguish between frontiers and what is not frontiers.
// If a point outside our manually set radius is hit, it is marked a frontier on the same distance as the radius.
// Frontier cells is marked as occupied in the octree.

void FrontierExtraction::insertPointCloudScan(const tf::Point &sensorOriginTf, const PCLPointCloud &ground, const PCLPointCloud &nonground)
{
    point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);
    if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
    {
        ROS_ERROR_STREAM("Could not generate Key for origin " << sensorOrigin);
    }
    KeySet free_cells, occupied_cells;

    for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it)
    {
        point3d point(it->x, it->y, it->z);
        // Check if point is within our radius
        if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange))
        {

            // Add all point to the point as free cells
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
            {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            // Add the point as occupied cell
            octomap::OcTreeKey key;
            if (m_octree->coordToKeyChecked(point, key))
            {
                occupied_cells.insert(key);
                updateMinKey(key, m_updateBBXMin);
                updateMaxKey(key, m_updateBBXMax);
            }
        }
        else
        {
            point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay))
            {
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                octomap::OcTreeKey endKey;
                if (m_octree->coordToKeyChecked(new_end, endKey))
                {
                    free_cells.insert(endKey);
                    updateMinKey(endKey, m_updateBBXMin);
                    updateMaxKey(endKey, m_updateBBXMax);
                }
                else
                {
                    ROS_ERROR_STREAM("Could not generate Key for endpoint " << new_end);
                }
            }
        }
    }

    // mark free cells only if not seen occupied in this cloud
    for (KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
    {
        if (occupied_cells.find(*it) == occupied_cells.end())
        {
            m_octree->updateNode(*it, false);
        }
    }

    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++)
    {
        m_octree->updateNode(*it, true);
    }

    octomap::point3d minPt, maxPt;

    minPt = m_octree->keyToCoord(m_updateBBXMin);
    maxPt = m_octree->keyToCoord(m_updateBBXMax);
}

void FrontierExtraction::extractFrontiers(const tf::Point &sensorOriginTf, const ros::Time &rostime)
{
    point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

    // Go through the old list of Frontier keys and check if they are still frontiers
    for (octomap::point3d_list::iterator it = frontierPointList.begin(), end = frontierPointList.end(); it != end; ++it)
    {

        float x = it->x();
        float y = it->y();
        float z = it->z();
        if (!FrontierExtraction::verifyFrontier(x, y, z, sensorOrigin))
        {
            it = frontierPointList.erase(it);
        }
    }

    // Go through all changed keys, add them to the frontier list if they are concidered to be a frontier.
    for (KeyBoolMap::const_iterator it = m_octree->changedKeysBegin(), end = m_octree->changedKeysEnd(); it != end; ++it)
    {
        if (it->second == true)
        {
            octomap::OcTreeNode *node = m_octree->search(it->first, 0);
            if (node != NULL)
            {
                if (!m_octree->isNodeOccupied(node))
                {
                    point3d point = m_octree->keyToCoord(it->first);
                    float x = point.x();
                    float y = point.y();
                    float z = point.z();
                    octomap::OcTreeNode *me;
                    me = m_octree->search(x, y, z);

                    point3d pt;
                    pt.x() = point.x();
                    pt.y() = point.y();
                    pt.z() = point.z();

                    double z_diff = abs(m_currentPose.pose.pose.position.z - pt.z());

                    if (z_diff < max_allowed_height_)
                    {

                        if (FrontierExtraction::verifyFrontier(x, y, z, sensorOrigin) && !(m_octree->isNodeOccupied(me)))
                        {
                            frontierPointList.push_back(pt);
                        }
                    }
                }
            }
        }
    }

    m_octree->resetChangeDetection();
}

std::vector<Eigen::Vector3d> FrontierExtraction::fetchFrontiers()
{

    // ROS_INFO_STREAM ("raw frontiers : " << frontierPointList.size() << "\n");

    std::vector<Eigen::Vector3d> frontiers;

    for (auto i = frontierPointList.begin(); i != frontierPointList.end(); i++)
    {

        Eigen::Vector3d frontier_pt(i->x(), i->y(), i->z());
      
        // VOXBLOX BOX STATUS FOR COLLISION CHECK
      
        if (MapManager::VoxelStatus::kFree ==
            map_manager_->getBoxStatus(frontier_pt + safety_margin_, robot_box_size_, true))
        {
          frontiers.push_back(frontier_pt);
        }

    }

    // ROS_INFO_STREAM ("Unfiltered frontiers size : " << unfiltered_frontiers.size() << "\n");

    return frontiers;
}

Eigen::Vector3d FrontierExtraction::quaternion_to_rpy(Eigen::Quaterniond q)
{
    const double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                              1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    const double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
    const double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                             1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    return Eigen::Vector3d{roll, pitch, yaw};
}

void FrontierExtraction::odomCallback(const nav_msgs::Odometry::ConstPtr &pose)
{

    if (firstTime)
    {
        m_prevPose = *pose;
        firstTime = false;
    }
    else
    {
        m_currentPose = *pose;
    }

    Eigen::Quaterniond attitude{pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
                                pose->pose.pose.orientation.y, pose->pose.pose.orientation.z};

    Eigen::Vector3d euler = quaternion_to_rpy(attitude);

    Eigen::Quaterniond Q(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()));

    quat_ = Q;

    tf::Quaternion q(
        pose->pose.pose.orientation.x,
        pose->pose.pose.orientation.y,
        pose->pose.pose.orientation.z,
        pose->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    robotState_[0] = pose->pose.pose.position.x;
    robotState_[1] = pose->pose.pose.position.y;
    robotState_[2] = pose->pose.pose.position.z;

    visualize();
}

void FrontierExtraction::wrt_Body(Eigen::Vector3d frontier_W, Eigen::Vector3d &frontier_B)
{

    frontier_B = quat_.toRotationMatrix().transpose() * frontier_W;
}

bool FrontierExtraction::verifyFrontier(float x, float y, float z, point3d sensorOrigin)
{

    octomap::OcTreeNode *me;
    octomap::OcTreeNode *nodeX;
    octomap::OcTreeNode *nodeY;
    octomap::OcTreeNode *nodeZ;
    octomap::OcTreeNode *nodeNegativeX;
    octomap::OcTreeNode *nodeNegativeY;
    octomap::OcTreeNode *nodeNegativeZ;
    point3d frontier(x, y, z);
    me = m_octree->search(x, y, z);
    if (m_octree->isNodeOccupied(me))
    {
        return false;
    }
    else
    {
        // North
        octomap::OcTreeNode *N = m_octree->search(x + (m_res), y, z, 0);
        octomap::OcTreeNode *NW = m_octree->search(x + (m_res), y + (m_res), z, 0);
        octomap::OcTreeNode *NE = m_octree->search(x + (m_res), y - (m_res), z, 0);
        octomap::OcTreeNode *NT = m_octree->search(x + (m_res), y, z + (m_res), 0);
        octomap::OcTreeNode *NTW = m_octree->search(x + (m_res), y + (m_res), z + (m_res), 0);
        octomap::OcTreeNode *NTE = m_octree->search(x + (m_res), y - (m_res), z + (m_res), 0);
        octomap::OcTreeNode *NB = m_octree->search(x + (m_res), y, z - (m_res), 0);
        octomap::OcTreeNode *NBW = m_octree->search(x + (m_res), y + (m_res), z - (m_res), 0);
        octomap::OcTreeNode *NBE = m_octree->search(x + (m_res), y - (m_res), z - (m_res), 0);
        // Mid
        octomap::OcTreeNode *W = m_octree->search(x, y + (m_res), z, 0);
        octomap::OcTreeNode *E = m_octree->search(x, y - (m_res), z, 0);
        octomap::OcTreeNode *T = m_octree->search(x, y, z + (m_res), 0);
        octomap::OcTreeNode *TW = m_octree->search(x, y + (m_res), z + (m_res), 0);
        octomap::OcTreeNode *TE = m_octree->search(x, y - (m_res), z + (m_res), 0);
        octomap::OcTreeNode *B = m_octree->search(x, y, z - (m_res), 0);
        octomap::OcTreeNode *BW = m_octree->search(x, y + (m_res), z - (m_res), 0);
        octomap::OcTreeNode *BE = m_octree->search(x, y - (m_res), z - (m_res), 0);
        // South
        octomap::OcTreeNode *S = m_octree->search(x - (m_res), y, z, 0);
        octomap::OcTreeNode *SW = m_octree->search(x - (m_res), y + (m_res), z, 0);
        octomap::OcTreeNode *SE = m_octree->search(x - (m_res), y - (m_res), z, 0);
        octomap::OcTreeNode *ST = m_octree->search(x - (m_res), y, z + (m_res), 0);
        octomap::OcTreeNode *STW = m_octree->search(x - (m_res), y + (m_res), z + (m_res), 0);
        octomap::OcTreeNode *STE = m_octree->search(x - (m_res), y - (m_res), z + (m_res), 0);
        octomap::OcTreeNode *SB = m_octree->search(x - (m_res), y, z - (m_res), 0);
        octomap::OcTreeNode *SBW = m_octree->search(x - (m_res), y + (m_res), z - (m_res), 0);
        octomap::OcTreeNode *SBE = m_octree->search(x - (m_res), y - (m_res), z - (m_res), 0);

        int points = 0;

        if (!N)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(N))
            {
                return false;
            }
        }
        if (!NW)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(NW))
            {
                return false;
            }
        }
        if (!NE)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(NE))
            {
                return false;
            }
        }
        if (!NT)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(NT))
            {
                return false;
            }
        }
        if (!NTW)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(NTW))
            {
                return false;
            }
        }
        if (!NTE)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(NTE))
            {
                return false;
            }
        }
        if (!NB)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(NB))
            {
                return false;
            }
        }
        if (!NBW)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(NBW))
            {
                return false;
            }
        }
        if (!NBE)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(NBE))
            {
                return false;
            }
        }

        if (!W)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(W))
            {
                return false;
            }
        }
        if (!E)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(E))
            {
                return false;
            }
        }
        if (!T)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(T))
            {
                return false;
            }
        }
        if (!TW)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(TW))
            {
                return false;
            }
        }
        if (!TE)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(TE))
            {
                return false;
            }
        }
        if (!B)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(B))
            {
                return false;
            }
        }
        if (!BW)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(BW))
            {
                return false;
            }
        }
        if (!BE)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(BE))
            {
                return false;
            }
        }

        if (!S)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(S))
            {
                return false;
            }
        }
        if (!SW)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(SW))
            {
                return false;
            }
        }
        if (!SE)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(SE))
            {
                return false;
            }
        }
        if (!ST)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(ST))
            {
                return false;
            }
        }
        if (!STW)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(STW))
            {
                return false;
            }
        }
        if (!STE)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(STE))
            {
                return false;
            }
        }
        if (!SB)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(SB))
            {
                return false;
            }
        }
        if (!SBW)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(SBW))
            {
                return false;
            }
        }
        if (!SBE)
        {
            points++;
        }
        else
        {
            if (m_octree->isNodeOccupied(SBE))
            {
                return false;
            }
        }

        float distance = (sensorOrigin - frontier).norm();

        if (points >= numberOfUnknowns && distance > m_distance_deletion)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

// Distance between two points in 3D

double FrontierExtraction::calculateDistance(Eigen::Vector3d point1, Eigen::Vector3d point2)
{

    double d = sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) + pow(point1.z() - point2.z(), 2));
    return d;
}

// Calculate relative Angle of a point wrt robot's current heading vector

double FrontierExtraction::calculateAngle(Eigen::Vector3d fpoint)
{
    return atan2((fpoint.y() - robotState_[1]), (fpoint.x() - robotState_[0])) * (180 / M_PI) - robotState_[3];
}

void FrontierExtraction::visualize()
{

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker frontiers_marker;
    visualization_msgs::Marker info_gain_marker;

    int marker_id = 0;

    for (auto i : fetchFrontiers())
    {

        frontiers_marker.header.frame_id = "world";
        frontiers_marker.header.stamp = ros::Time::now();
        frontiers_marker.ns = "frontier";
        frontiers_marker.id = 0;
        frontiers_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        frontiers_marker.action = visualization_msgs::Marker::ADD;
        frontiers_marker.pose.orientation.w = 1;
        frontiers_marker.scale.x = 0.3;
        frontiers_marker.scale.y = 0.3;
        frontiers_marker.scale.z = 0.3;
        frontiers_marker.color.a = 0.7;
        frontiers_marker.color.r = 1.0;
        frontiers_marker.color.g = 0.0;
        frontiers_marker.color.b = 0.0;

        geometry_msgs::Point p;
        p.x = i[0];
        p.y = i[1];
        p.z = i[2];
        frontiers_marker.points.push_back(p);
        marker_array.markers.push_back(frontiers_marker);
    }

    frontiersVisPub_.publish(marker_array);
}

double FrontierExtraction::findAngle(Eigen::Vector3d fpoint)
{

    Eigen::Vector3d frontier_W;
    frontier_W[0] = fpoint.x();
    frontier_W[1] = fpoint.y();
    frontier_W[2] = fpoint.z();

    Eigen::Vector3d frontier_B_;

    wrt_Body(frontier_W, frontier_B_);

    double angle = atan2(frontier_B_[1] - robotState_[1], frontier_B_[0] - robotState_[0]) * 180 / M_PI;

    return angle;
    // cout << "Eigen angle : " << angle << "\n";
}

// If the point is in FOV and in local range, it is marked as local wrt the robot

bool FrontierExtraction::isLocal(Eigen::Vector3d point)
{

    double angle = calculateAngle(point);
    double distance = calculateDistance(point, m_currentPosition);

    if (((angle < LOCAL_FOV) && (angle > -LOCAL_FOV)) && distance < LOCAL_SPACE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Extract frontiers that are in FOV and in local range

std::vector<Eigen::Vector3d> FrontierExtraction::fetchLocalFrontiers()
{

    std::vector<Eigen::Vector3d> local_frontiers;
    std::vector<Eigen::Vector3d> frontiers = fetchFrontiers();

    for (Eigen::Vector3d frontier_point : frontiers)
    {

        if (isLocal(frontier_point))
        {

            local_frontiers.push_back(frontier_point);
        }
    }

    // ROS_INFO_STREAM("local frontiers size " << local_frontiers.size());

    return local_frontiers;
}

// Extract frontiers that are outside FOV and not in local range

std::vector<Eigen::Vector3d> FrontierExtraction::fetchGlobalFrontiers()
{

    std::vector<Eigen::Vector3d> global_frontiers;
    std::vector<Eigen::Vector3d> frontiers = fetchFrontiers();

    for (Eigen::Vector3d frontier_point : frontiers)
    {

        if (!isLocal(frontier_point))
        {

            global_frontiers.push_back(frontier_point);
        }
    }

    return global_frontiers;
}

// conversion utility

geometry_msgs::Point FrontierExtraction::eigenToPoint(Eigen::Vector3d v)
{
    geometry_msgs::Point p;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    return p;
}

std::vector<geometry_msgs::Point> FrontierExtraction::fetchLocalTargets()
{

    std::vector<geometry_msgs::Point> local_targets;

    for (Eigen::Vector3d local_frontier : fetchLocalFrontiers())
    {

        local_targets.push_back(eigenToPoint(local_frontier));
    }

    return local_targets;
}
// If no local frontiers, fetch re-positioning targets for global exploration

std::vector<geometry_msgs::Point> FrontierExtraction::fetchGlobalRepositioningTargets()
{

    std::vector<geometry_msgs::Point> global_repos_targets;

    for (Eigen::Vector3d global_frontier : fetchGlobalFrontiers())
    {

        global_repos_targets.push_back(eigenToPoint(global_frontier));
    }

    return global_repos_targets;
}

// Export exploration target (local or global depending on request from graph management)

bool FrontierExtraction::exportFrontierTargets(stage_planner::frontierRequest::Request &req,
                                               stage_planner::frontierRequest::Response &res)
{

    if (req.explore)
    {

        res.local_targets = fetchLocalTargets();

        res.global_targets = fetchGlobalRepositioningTargets();

        if (res.local_targets.empty() and res.global_targets.empty())
        {
            res.frontiers_exist = false;
        }
        else
        {
            res.frontiers_exist = true;
        }
    }

    return true;
}

bool FrontierExtraction::queryFrontierTarget(stage_planner::isFrontier::Request &req,
                                             stage_planner::isFrontier::Response &res)
{

    octomap::point3d frontier_in_octree(req.frontier_point.x, req.frontier_point.y,
                                        req.frontier_point.z);

    if (std::find(frontierPointList.begin(), frontierPointList.end(),
                  frontier_in_octree) != frontierPointList.end())
    {
        res.is_frontier = true;
    }
    else
    {
        res.is_frontier = false;
    }

    return true;
}
