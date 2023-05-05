#include "frontiers_extraction.h"

using namespace octomap;
using namespace std;

using namespace std::chrono;
using namespace std::this_thread;

FrontierExtraction::FrontierExtraction(ros::NodeHandle private_nh_)
    : m_nh(),
    m_maxRange(10.0),
    m_filterGroundPlane(false),
    m_colorFactor(0.8),
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
    m_odometry("spot/ground_truth"),
    cloud_in("/camera/depth/color/points"),
    m_res(0.5),
    m_treeDepth(0),
    m_maxTreeDepth(0),
    xFiltering(false),
    yFiltering(false),
    zFiltering(false),
    m_distance_deletion(8.0),
    m_currentHeading(0),
    LOCAL_SPACE(10),
    LOCAL_FOV(40),    
    m_ufomap("/ufomap_mapping_server_node/map"),
    m_max_allowed_Z(4),  
    OBB_RADIUS(0.5),
    PLANNING_CHECK_DEPTH(0),
    VOLUMETRIC_GAIN_THREASHOLD(10)




{


   

    double probHit, probMiss, thresMin, thresMax;

    ros::NodeHandle private_nh = private_nh_;
    //Set variables
    private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
    private_nh.param("resolution", m_res, m_res);
    private_nh.param("sensor_model/hit", probHit, 0.7);
    private_nh.param("sensor_model/miss", probMiss, 0.4);
    private_nh.param("sensor_model/min", thresMin, 0.12);
    private_nh.param("sensor_model/max", thresMax, 0.97);
    private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);
    private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
    private_nh.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
    private_nh.param("color_factor", m_colorFactor, m_colorFactor);
    private_nh.param("cloud_in", cloud_in, cloud_in);
    private_nh.param("distance", m_distance_deletion, m_distance_deletion);
    private_nh.param("odometry", m_odometry, m_odometry);
    
    private_nh.param("number_of_unknowns", numberOfUnknowns, numberOfUnknowns);

    private_nh.param("local_sliding_space", LOCAL_SPACE, LOCAL_SPACE);
    private_nh.param("local_field_of_view", LOCAL_FOV, LOCAL_SPACE);
    private_nh.param("ufomap", m_ufomap, m_ufomap);
    private_nh.param("safety_height", m_max_allowed_Z, m_max_allowed_Z);
    private_nh.param("traversability_radius", OBB_RADIUS, OBB_RADIUS);
    private_nh.param("planning_check_depth", PLANNING_CHECK_DEPTH, PLANNING_CHECK_DEPTH);
    private_nh.param("volumetric_gain_threashold", VOLUMETRIC_GAIN_THREASHOLD, VOLUMETRIC_GAIN_THREASHOLD);






    //Filtering
    private_nh.param("xFiltering", xFiltering, xFiltering);
    private_nh.param("yFiltering", yFiltering, yFiltering);
    private_nh.param("zFiltering", zFiltering, zFiltering);

    //Setup octree
    m_octree = new OcTreeT(m_res);
    m_octree->setProbHit(probHit);
    m_octree->setProbMiss(probMiss);
    m_octree->setClampingThresMin(thresMin);
    m_octree->setClampingThresMax(thresMax);
    m_octree->enableChangeDetection(true);
    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;

    double r, g, b, a;
    private_nh.param("color/r", r, 0.0);
    private_nh.param("color/g", g, 0.0);
    private_nh.param("color/b", b, 1.0);
    private_nh.param("color/a", a, 1.0);
    m_color.r = r;
    m_color.g = g;
    m_color.b = b;
    m_color.a = a;

    private_nh.param("color_free/r", r, 0.0);
    private_nh.param("color_free/g", g, 1.0);
    private_nh.param("color_free/b", b, 0.0);
    private_nh.param("color_free/a", a, 1.0);
    m_colorFree.r = r;
    m_colorFree.g = g;
    m_colorFree.b = b;
    m_colorFree.a = a;

    //Register to the subscriber topic and register it to the callback function
    m_pointCloudSubscriber = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, cloud_in, 5);
    m_tfPointCloudSubscriber = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSubscriber, m_tfListener, m_worldFrameId, 5);
    m_tfPointCloudSubscriber->registerCallback(boost::bind(&FrontierExtraction::insertPointCloudCallback, this, _1));

    m_poseSubscriber = new message_filters::Subscriber<nav_msgs::Odometry> (m_nh, m_odometry, 5);
    m_poseSubscriber->registerCallback(boost::bind(&FrontierExtraction::updateCurrentPosition, this, _1));

    m_ufoMapSubscriber = new message_filters::Subscriber<ufomap_msgs::UFOMapStamped> (m_nh, m_ufomap, 1);
    m_ufoMapSubscriber->registerCallback(boost::bind(&FrontierExtraction::mapCallback, this, _1));

    //Register publishers
    m_frontierPub = m_nh.advertise<visualization_msgs::MarkerArray>("/frontier_nodes_vis", 1, true);
    // m_frontierSelectionPub = m_nh.advertise<Frontiers>("frontiers_nodes", 1, true);
};

 ufo::map::OccupancyMapColor m_ufoMAP(0.2);

void FrontierExtraction::mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg)
{
 
  if (ufomap_msgs::msgToUfo(msg->map, m_ufoMAP))
  {
    ROS_INFO("MAP RECEIVED");
    map_received = true;
  }
  else
  {
    ROS_WARN("MAP NOT RECEIVED");
  }
}


void FrontierExtraction::insertPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    PCLPointCloud pc;
    pcl::fromROSMsg(*cloud, pc);

    tf::StampedTransform sensorToWorldTf;
    try {
        m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex) {
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

/*
This function distingguish between frontiers and what is not frontiers.
If a point outside our manually set radius is hit, it is marked a frontier on the same distance as the radius.
Use Rviz for visualization.
** NOTICE **
Frontier cells is marked as occupied in the octree.
*/
void FrontierExtraction::insertPointCloudScan(const tf::Point& sensorOriginTf, const PCLPointCloud& ground, const PCLPointCloud& nonground) {
    point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);
    if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {
        ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
    }
    KeySet free_cells, occupied_cells;

    for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it) {
        point3d point(it->x, it->y, it->z);
        //Check if point is within our radius
        if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

            // Add all point to the point as free cells
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            // Add the point as occupied cell
            octomap::OcTreeKey key;
            if (m_octree->coordToKeyChecked(point, key)){
                occupied_cells.insert(key);
                updateMinKey(key, m_updateBBXMin);
                updateMaxKey(key, m_updateBBXMax);
            }
        } else {
            point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                octomap::OcTreeKey endKey;
                if (m_octree->coordToKeyChecked(new_end, endKey)){
                    free_cells.insert(endKey);
                    updateMinKey(endKey, m_updateBBXMin);
                    updateMaxKey(endKey, m_updateBBXMax);
                } else{
                ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
                }
            }
        }
    }



    // mark free cells only if not seen occupied in this cloud
    for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
        if (occupied_cells.find(*it) == occupied_cells.end()){
            m_octree->updateNode(*it, false);
        }
    }

    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
        m_octree->updateNode(*it, true);
    }

    octomap::point3d minPt, maxPt;

    minPt = m_octree->keyToCoord(m_updateBBXMin);
    maxPt = m_octree->keyToCoord(m_updateBBXMax);
}



void FrontierExtraction::extractFrontiers(const tf::Point& sensorOriginTf, const ros::Time& rostime) {
    point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

    //Go through the old list of Frontier keys and check if they are still frontiers
    for (octomap::point3d_list::iterator it = frontierPointList.begin(), end = frontierPointList.end(); it != end; ++it) {

        float x = it->x();
        float y = it->y();
        float z = it->z();
        if (!FrontierExtraction::isFrontier(x, y, z, sensorOrigin)) {
            it = frontierPointList.erase(it);
        }
    }

    //Go through all changed keys, add them to the frontier list if they are concidered to be a frontier.
    for(KeyBoolMap::const_iterator it = m_octree->changedKeysBegin(), end = m_octree->changedKeysEnd(); it != end; ++it) {
        if(it->second == true){
            octomap::OcTreeNode* node = m_octree->search(it->first,0);
            if(node != NULL) {
                if (!m_octree->isNodeOccupied(node)) {
                    point3d point = m_octree->keyToCoord(it->first);
                    float x = point.x();
                    float y = point.y();
                    float z = point.z();
                    octomap::OcTreeNode* me;
                    me = m_octree->search(x,y,z);

                    point3d pt;
                    pt.x() = point.x();
                    pt.y() = point.y();
                    pt.z() = point.z();
                    
                    double z_diff = abs(m_currentPose.pose.pose.position.z - pt.z());

                    if ((z_diff < m_max_allowed_Z)) {     
                        
                        if (FrontierExtraction::isFrontier(x, y, z, sensorOrigin) && !(m_octree->isNodeOccupied(me))) {
                            frontierPointList.push_back(pt);
                        }
                }
                    
                }
            }
        }
    }

    m_octree->resetChangeDetection();
}

std::vector<ufo::math::Vector3> FrontierExtraction::fetchFrontiers () {

  std::vector<ufo::math::Vector3> unfiltered_frontiers;

  for (auto i = frontierPointList.begin(); i != frontierPointList.end(); i++) {

    ufo::math::Vector3 frontier_pt (i->x(), i->y(), i->z());
    
    ufo::geometry::Sphere sphere (frontier_pt, 0.5);

    if (!isInCollision(m_ufoMAP, sphere, true, false, true, PLANNING_CHECK_DEPTH) && computeVolumetricGain(frontier_pt, m_ufoMAP) > VOLUMETRIC_GAIN_THREASHOLD) {

        unfiltered_frontiers.push_back(frontier_pt);
    }

  }

  return unfiltered_frontiers;
}


void FrontierExtraction::updateCurrentPosition(const nav_msgs::Odometry::ConstPtr& pose) {

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

ufo::math::Vector3 FrontierExtraction::fetchCurrentPosition () {
  return m_currentPosition;
}

bool FrontierExtraction::isFrontier(float x, float y, float z, point3d sensorOrigin) {


    octomap::OcTreeNode* me;
    octomap::OcTreeNode* nodeX;
    octomap::OcTreeNode* nodeY;
    octomap::OcTreeNode* nodeZ;
    octomap::OcTreeNode* nodeNegativeX;
    octomap::OcTreeNode* nodeNegativeY;
    octomap::OcTreeNode* nodeNegativeZ;
    point3d frontier(x,y,z);
    me = m_octree->search(x,y,z);
    if(m_octree->isNodeOccupied(me))
    {
        return false;
    }
    else {
        //North
        octomap::OcTreeNode* N = m_octree->search(x+(m_res ),y,z,0);
        octomap::OcTreeNode* NW = m_octree->search(x+(m_res ),y+(m_res ),z,0);
        octomap::OcTreeNode* NE = m_octree->search(x+(m_res ),y-(m_res ), z,0);
        octomap::OcTreeNode* NT = m_octree->search(x+(m_res ),y,z+(m_res ),0);
        octomap::OcTreeNode* NTW = m_octree->search(x+(m_res ),y+(m_res ),z+(m_res ),0);
        octomap::OcTreeNode* NTE = m_octree->search(x+(m_res ),y-(m_res ), z+(m_res ),0);
        octomap::OcTreeNode* NB = m_octree->search(x+(m_res ),y,z-(m_res ),0);
        octomap::OcTreeNode* NBW = m_octree->search(x+(m_res ),y+(m_res ),z-(m_res ),0);
        octomap::OcTreeNode* NBE = m_octree->search(x+(m_res ),y-(m_res ), z-(m_res ),0);
        //Mid
        octomap::OcTreeNode* W = m_octree->search(x,y+(m_res ),z,0);
        octomap::OcTreeNode* E = m_octree->search(x,y-(m_res ), z,0);
        octomap::OcTreeNode* T = m_octree->search(x,y,z+(m_res ),0);
        octomap::OcTreeNode* TW = m_octree->search(x,y+(m_res ),z+(m_res ),0);
        octomap::OcTreeNode* TE = m_octree->search(x,y-(m_res ), z+(m_res ),0);
        octomap::OcTreeNode* B = m_octree->search(x,y,z-(m_res ),0);
        octomap::OcTreeNode* BW = m_octree->search(x,y+(m_res ),z-(m_res ),0);
        octomap::OcTreeNode* BE = m_octree->search(x,y-(m_res ), z-(m_res ),0);
        //South
        octomap::OcTreeNode* S = m_octree->search(x-(m_res ),y,z,0);
        octomap::OcTreeNode* SW = m_octree->search(x-(m_res ),y+(m_res ),z,0);
        octomap::OcTreeNode* SE = m_octree->search(x-(m_res ),y-(m_res ), z,0);
        octomap::OcTreeNode* ST = m_octree->search(x-(m_res ),y,z+(m_res ),0);
        octomap::OcTreeNode* STW = m_octree->search(x-(m_res ),y+(m_res ),z+(m_res ),0);
        octomap::OcTreeNode* STE = m_octree->search(x-(m_res ),y-(m_res ), z+(m_res ),0);
        octomap::OcTreeNode* SB = m_octree->search(x-(m_res ),y,z-(m_res ),0);
        octomap::OcTreeNode* SBW = m_octree->search(x-(m_res ),y+(m_res ),z-(m_res ),0);
        octomap::OcTreeNode* SBE = m_octree->search(x-(m_res ),y-(m_res ), z-(m_res ),0);

        int points = 0;

        if (!N) {
            points++;
        } else {
            if(m_octree->isNodeOccupied(N)) {
                return false;
            }
        }
        if (!NW) {
            points++;
        }
        else {
            if(m_octree->isNodeOccupied(NW)) {
                return false;
            }
        }
        if (!NE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NE)) {
                return false;
            }
        }
        if (!NT) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NT)) {
                return false;
            }
        }
        if (!NTW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NTW)) {
                return false;
            }
        }
        if (!NTE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NTE)) {
                return false;
            }
        }
        if (!NB) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NB)) {
                return false;
            }
        }
        if (!NBW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NBW)) {
                return false;
            }
        }
        if (!NBE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NBE)) {
                return false;
            }
        }

        if (!W) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(W)) {
                return false;
            }
        }
        if (!E) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(E)) {
                return false;
            }
        }
        if (!T) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(T)) {
                return false;
            }
        }
        if (!TW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(TW)) {
                return false;
            }
        }
        if (!TE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(TE)) {
                return false;
            }
        }
        if (!B) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(B)) {
                return false;
            }
        }
        if (!BW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(BW)) {
                return false;
            }
        }
        if (!BE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(BE)) {
                return false;
            }
        }


        if (!S) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(S)) {
                return false;
            }
        }
        if (!SW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SW)) {
                return false;
            }
        }
        if (!SE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SE)) {
                return false;
            }
        }
        if (!ST) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(ST)) {
                return false;
            }
        }
        if (!STW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(STW)) {
                return false;
            }
        }
        if (!STE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(STE)) {
                return false;
            }
        }
        if (!SB) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SB)) {
                return false;
            }
        }
        if (!SBW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SBW)) {
                return false;
            }
        }
        if (!SBE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SBE)) {
                return false;
            }
        }

    float distance = (sensorOrigin - frontier).norm();


    if (points >= numberOfUnknowns && distance > m_distance_deletion ) {
        return true;
    } else {
        return false;
    }
}


}

double FrontierExtraction::calculateDistance (ufo::math::Vector3 point1, ufo::math::Vector3 point2) {

    double d = sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) + pow(point1.z() - point2.z(), 2));
    return d;

}

bool FrontierExtraction::isInCollision(ufo::map::OccupancyMapColor const &map,
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

ufo::geometry::OBB FrontierExtraction::makeOBB(ufo::math::Vector3 source, ufo::math::Vector3 goal, float radius)
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

double FrontierExtraction::calculateAngle(ufo::math::Vector3 fpoint, ufo::math::Vector3 position,
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

bool FrontierExtraction::isLocal (ufo::math::Vector3 point) {

    double angle = FrontierExtraction::calculateAngle (point, m_currentPosition, m_currentHeading);
    double distance = FrontierExtraction::calculateDistance (point, m_currentPosition);

    if ((angle < LOCAL_FOV && angle > LOCAL_FOV) && distance < LOCAL_SPACE) {
        return true;
    } else { return false; }

}

std::vector<ufo::math::Vector3> FrontierExtraction::fetchLocalFrontiers () {


  std::vector<ufo::math::Vector3> local_frontiers;
  std::vector<ufo::math::Vector3> frontiers = fetchFrontiers();

  for (ufo::math::Vector3 frontier_point : frontiers) {
  
    if (isLocal(frontier_point)) {

      local_frontiers.push_back(frontier_point);

    }

  }
    
    // ROS_INFO_STREAM("local frontiers size " << local_frontiers.size());

  return local_frontiers;
}

std::vector<ufo::math::Vector3> FrontierExtraction::fetchGlobalFrontiers () {

  std::vector<ufo::math::Vector3> global_frontiers;
  std::vector<ufo::math::Vector3> frontiers = fetchFrontiers();

  for (ufo::math::Vector3 frontier_point : frontiers) {
  
    if (!isLocal(frontier_point)) {

      global_frontiers.push_back(frontier_point);

    }

  }

  return global_frontiers;
}

std::vector<ufo::math::Vector3> FrontierExtraction::fetchLocallyTraversableFrontiers () {

  std::vector<ufo::math::Vector3> locally_traversable_frontiers;

  for (ufo::math::Vector3 local_frontier : fetchLocalFrontiers() ) {

  ufo::geometry::OBB obb = makeOBB (fetchCurrentPosition(), local_frontier, OBB_RADIUS);

  if (!isInCollision(m_ufoMAP, obb, true, false, true, PLANNING_CHECK_DEPTH)) {

      locally_traversable_frontiers.push_back(local_frontier);

    }
  }
  return locally_traversable_frontiers;
}

double FrontierExtraction::computeVolumetricGain (ufo::math::Vector3 frontier, ufo::map::OccupancyMapColor const &map) {
    
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

ufo::math::Vector3 FrontierExtraction::fetchInformativeFrontier () {

    ufo::math::Vector3 informative_frontier;

    double total_cost = std::numeric_limits<double>::max();
    double cost;

    for (ufo::math::Vector3 frontier : fetchLocalFrontiers()) {

        cost = calculateAngle(frontier, fetchCurrentPosition(), m_currentHeading) + (1/computeVolumetricGain(frontier, m_ufoMAP));

        if (cost < total_cost) {
            total_cost = cost;
            informative_frontier = frontier;
        }
    }
    return informative_frontier;

}

std::vector< std::pair<ufo::math::Vector3, double>> FrontierExtraction::fetchGlobalRepositioningTargets() {

    std::vector< std::pair<ufo::math::Vector3, double>> global_repos_targets;
    
    for (ufo::math::Vector3 global_frontier : fetchGlobalFrontiers() ) {

        double volumetric_gain = computeVolumetricGain(global_frontier, m_ufoMAP);

        std::pair<ufo::math::Vector3, double> frontier_w_gain = std::make_pair (global_frontier, volumetric_gain);

        global_repos_targets.push_back(frontier_w_gain);

    }

    // std::cout << "global repos target infogain " << global_repos_targets.back().second << std::endl;

    return global_repos_targets;
}


    

