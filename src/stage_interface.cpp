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

void StageInterface::odomCallback(const nav_msgs::Odometry::ConstPtr &pose)
{

    m_currentPose = *pose;
    // std::cout << "current position X " << m_currentPose.pose.pose.position.x << std::endl;
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

void StageInterface::segmentPath(const std::vector<geometry_msgs::Pose>& path,
                                 std::vector<geometry_msgs::Pose>& path_seg) {
  path_seg.clear();
  double v_max_ = 0.5;
  double yaw_rate_max_ = 0.5;
  if (path.size() == 0) return;
  if (path.size() == 1) path_seg.push_back(path[0]);

  for (int i = 0; i < (path.size() - 1); ++i) {
    // Interpolate each segment.
    Eigen::Vector3d start(path[i].position.x, path[i].position.y,
                          path[i].position.z);
    Eigen::Vector3d end(path[i + 1].position.x, path[i + 1].position.y,
                        path[i + 1].position.z);
    Eigen::Vector3d distance = end - start;
    double yaw_start = tf::getYaw(path[i].orientation);
    double yaw_end = tf::getYaw(path[i + 1].orientation);
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
        path_seg.push_back(pose);
      }
    }
  }
} // segmentPath

void StageInterface::executeCurrentPath() {

    std::vector<geometry_msgs::Pose> raw_path;
    raw_path.clear();
    
    // TODO Insert current position at the begining of exploration path to get smooth start of next path following action.

    for (int vert_it = 0; vert_it < exploration_path_.poses.size(); vert_it++)
    { 
      
      ufo::math::Vector3 prev(exploration_path_.poses[vert_it].x, exploration_path_.poses[vert_it].y, exploration_path_.poses[vert_it].z);
      ufo::math::Vector3 next(exploration_path_.poses[vert_it+1].x, exploration_path_.poses[vert_it+1].y, exploration_path_.poses[vert_it+1].z);
      ufo::math::Vector3 before_last(exploration_path_.poses[vert_it-1].x, exploration_path_.poses[vert_it-1].y, exploration_path_.poses[vert_it-1].z);

     
      float angle = atan2( next.y() - prev.y(), next.x() - prev.x());
      float angle_last = atan2( prev.y() - before_last.y(), prev.x() - before_last.x());
      ufo::math::Vector3 v(0, 0, angle);
      ufo::math::Vector3 v_last(0, 0, angle_last);
      Eigen::Quaternion<float> q = Euler2Quaternion(v);
      Eigen::Quaternion<float> q_last = Euler2Quaternion(v_last);

      if (vert_it == exploration_path_.poses.size()-1) {
        q = q_last;
      }
      
      geometry_msgs::Pose raw_path_pose;
      raw_path_pose.position.x = exploration_path_.poses[vert_it].x;
      raw_path_pose.position.y = exploration_path_.poses[vert_it].y;
      raw_path_pose.position.z = exploration_path_.poses[vert_it].z;
      raw_path_pose.orientation.x = q.x();
      raw_path_pose.orientation.y = q.y();
      raw_path_pose.orientation.z = q.z();
      raw_path_pose.orientation.w = q.w();
      raw_path.push_back(raw_path_pose);
      
    }

    std::vector<geometry_msgs::Pose> new_path;
    new_path.clear();
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

    current.temppath.poses.clear();
    current.path.path.poses.clear();
    current.path.type.data.clear();

    for (int i = 0; i < new_path.size(); i++) {
      
      geometry_msgs::PoseStamped pose_ref;

      double yaw = tf::getYaw(new_path[i].orientation);
      Eigen::Vector3d p(new_path[i].position.x, new_path[i].position.y,
                        new_path[i].position.z);
      trajectory_point_.position_W.x() = p.x();
      trajectory_point_.position_W.y() = p.y();
      trajectory_point_.position_W.z() = p.z();
      trajectory_point_.setFromYaw(yaw);
      pose_ref.pose.position.x = new_path[i].position.x;
      pose_ref.pose.position.y = new_path[i].position.y;
      pose_ref.pose.position.z = new_path[i].position.z;
      pose_ref.pose.orientation.x = new_path[i].orientation.x;
      pose_ref.pose.orientation.y = new_path[i].orientation.y;
      pose_ref.pose.orientation.z = new_path[i].orientation.z;
      pose_ref.pose.orientation.w = new_path[i].orientation.w;
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(
          trajectory_point_, &trajectory_point_msg_);
      time_sum += 2*dt_;
      trajectory_point_msg_.time_from_start = ros::Duration(time_sum);
      // trajectory_array_.points.insert(trajectory_array_.points.begin(),trajectory_point_msg_);
      trajectory_array_.points.push_back(trajectory_point_msg_);
      current.temppath.poses.push_back(pose_ref);
      command_path.poses.push_back(new_path[i]);

    }
    pub.trajectory_pub.publish(trajectory_array_);
    pub.posearray_pub.publish(command_path);

    ufo::math::Vector3 lastpub(std::prev(exploration_path_.poses.end(), 1)->x,
                               std::prev(exploration_path_.poses.end(), 1)->y,
                               std::prev(exploration_path_.poses.end(), 1)->z);
    published.reference = publish_goal(lastpub);

} // executeCurrentPath

