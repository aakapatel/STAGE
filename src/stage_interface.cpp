#include "stage_interface.h"

using namespace std;

StageInterface::StageInterface(ros::NodeHandle private_nh_)
    : m_nh(),
      m_odometry("/hummingbird/ground_truth/odometry")
// cloud_in("/hummingbird/velodyne_points"),
{

    ros::NodeHandle private_nh = private_nh_;
    // Set variables
    private_nh.param("odometry", m_odometry, m_odometry);

    // Register to the subscriber topic and register it to the callback function

    m_poseSubscriber = new message_filters::Subscriber<nav_msgs::Odometry>(m_nh, m_odometry, 5);
    m_poseSubscriber->registerCallback(boost::bind(&StageInterface::odomCallback, this, _1));

    // Register publishers



    // Ros services 
  
    
    m_exploration_path_client = m_nh.serviceClient<stage_planner::explorationPath>(
      "next_exploration_path");




};

void StageInterface::updateCurrentPosition (const nav_msgs::Odometry::ConstPtr &pose)
{

  if (firstTime)
  {
    home_position.x() = pose->pose.pose.position.x;
    home_position.y() = pose->pose.pose.position.y;
    home_position.z() = pose->pose.pose.position.z;
    m_prevPose = *pose;
    firstTime = false;
  }
  else
  {
    m_currentPose = *pose;
    //
    // ufo::math::Vector3 newPoint(pose->pose.pose.position.x, pose->pose.pose.position.y,
    //                             pose->pose.pose.position.z);
    // ufo::math::Vector3 oldPoint(m_prevPose.pose.pose.position.x, m_prevPose.pose.pose.position.y,
    //                             m_prevPose.pose.pose.position.z);
    // geometry_msgs::PoseWithCovariance old_pose = m_prevPose.pose;
    // geometry_msgs::PoseWithCovariance new_pose = m_currentPose.pose;
    // if (StageInterface::calculateDistance(newPoint, oldPoint) > 1)
    // {
    //   m_prevPose = *pose;
    // }
  // }

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

ufo::math::Vector3 StageInterface::fetchCurrentPosition()
{
  return m_currentPosition;
}

double StageInterface::calculateDistance(ufo::math::Vector3 point1,
                                       ufo::math::Vector3 point2)
{

  double d =
      sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) +
           pow(point1.z() - point2.z(), 2));
  return d;
}

bool StageInterface::fetchExplorationPath () {

  stage_planner::explorationPath exploration_path_srv;
  
  exploration_path_srv.request.exploration = true;

  if (m_exploration_path_client.call(exploration_path_srv)) {
    
    exploration_path_ = exploration_path_srv.response.path;

    planning_status_local = exploration_path_srv.response.planning_status_local;
    planning_status_global = exploration_path_srv.response.planning_status_global;    
    
    // TODO Add a ros_info_cond to log planning status 
  
  }

}

void StageInterface::segmentPath(const nav_msgs::Path& path,
                                 nav_msgs::Path& path_seg.poses) {
  path_seg.poses.clear();
  double v_max_ = 0.5;
  double yaw_rate_max_ = 0.5;
  if (path.poses.size() == 0) return;
  if (path.poses.size() == 1) path_seg.poses.push_back(path.poses[0]);

  for (int i = 0; i < (path.poses.size() - 1); ++i) {
    // Interpolate each segment.
    Eigen::Vector3d start(path.poses[i].position.x, path.poses[i].position.y,
                          path.poses[i].position.z);
    Eigen::Vector3d end(path.poses[i + 1].position.x, path.poses[i + 1].position.y,
                        path.poses[i + 1].position.z);
    Eigen::Vector3d distance = end - start;
    double yaw_start = tf::getYaw(path.poses[i].orientation);
    double yaw_end = tf::getYaw(path.poses[i + 1].orientation);
    double yaw_direction = yaw_end - yaw_start;
    if (yaw_direction > M_PI) {
      yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI) {
      yaw_direction += 2.0 * M_PI;
    }

    double dist_norm = distance.norm();
    double disc = std::min(dt_ * v_max_ / dist_norm,
                           dt_ * yaw_rate_max_ / abs(yaw_direction));

    bool int_flag = true;

    if (int_flag) {
      for (double it = 0.0; it <= 1.0; it += disc) {
        tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                           (1.0 - it) * start[1] + it * end[1],
                           (1.0 - it) * start[2] + it * end[2]);
        double yaw = yaw_start + yaw_direction * it;
        if (yaw > M_PI) yaw -= 2.0 * M_PI;
        if (yaw < -M_PI) yaw += 2.0 * M_PI;
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        tf::Pose poseTF(quat, origin);
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(poseTF, pose);
        path_seg.poses.push_back(pose);
      }
    }
  }
} // segmentPath

void StageInterface::executeCurrentPath() {

    nav_msgs::Path raw_path = exploration_path_;
    
    // TODO Insert current position at the begining of exploration path to get smooth start of next path following action.

    nav_msgs::Path new_path;
    new_path.poses.clear();
    segmentCurrentPath(raw_path, new_path);

    n_seq_++;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_array_;
    mav_msgs::EigenTrajectoryPoint trajectory_point_;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg_;
    std::vector<geometry_msgs::Pose> executing_path_;
    
    trajectory_array_.header.seq = n_seq_;
    trajectory_array_.header.stamp = ros::Time::now();
    trajectory_array_.header.frame_id = ODOM_FRAME_ID;
    trajectory_array_.points.clear();
    double time_sum = 0;
    geometry_msgs::PoseArray command_path;
    command_path.header.frame_id = ODOM_FRAME_ID;

    for (int i = 0; i < new_path.poses.size(); i++) {
      
      geometry_msgs::PoseStamped pose_ref;

      double yaw = tf::getYaw(new_path.poses[i].orientation);
      Eigen::Vector3d p(new_path.poses[i].position.x, new_path.poses[i].position.y,
                        new_path.poses[i].position.z);
      trajectory_point_.position_W.x() = p.x();
      trajectory_point_.position_W.y() = p.y();
      trajectory_point_.position_W.z() = p.z();
      trajectory_point_.setFromYaw(yaw);
      pose_ref.pose.position.x = new_path.poses[i].position.x;
      pose_ref.pose.position.y = new_path.poses[i].position.y;
      pose_ref.pose.position.z = new_path.poses[i].position.z;
      pose_ref.pose.orientation.x = new_path.poses[i].orientation.x;
      pose_ref.pose.orientation.y = new_path.poses[i].orientation.y;
      pose_ref.pose.orientation.z = new_path.poses[i].orientation.z;
      pose_ref.pose.orientation.w = new_path.poses[i].orientation.w;
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(
          trajectory_point_, &trajectory_point_msg_);
      time_sum += 2*dt_;
      trajectory_point_msg_.time_from_start = ros::Duration(time_sum);
      // trajectory_array_.points.insert(trajectory_array_.points.begin(),trajectory_point_msg_);
      trajectory_array_.points.push_back(trajectory_point_msg_);
      current.temppath.poses.push_back(pose_ref);
      command_path.poses.push_back(new_path.poses[i]);

    }
    pub.trajectory_pub.publish(trajectory_array_);
    pub.posearray_pub.publish(command_path);

    ufo::math::Vector3 lastpub(std::prev(exploration_path_.poses.end(), 1)->x,
                               std::prev(exploration_path_.poses.end(), 1)->y,
                               std::prev(exploration_path_.poses.end(), 1)->z);
    published.reference = publish_goal(lastpub);

} // executeCurrentPath

