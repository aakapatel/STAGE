#include "stage_interface.h"

using namespace std;

StageInterface::StageInterface(ros::NodeHandle private_nh_)
    : m_nh(),
      m_odometry("/hummingbird/ground_truth/odometry"),
      m_reference_topic("/hummingbird/command/pose")
{

    ros::NodeHandle private_nh = private_nh_;
    // Set variables
    private_nh.param("odometry", m_odometry, m_odometry);
    private_nh.param("reference_topic", m_reference_topic, m_reference_topic);

    // Register to the subscriber topic and register it to the callback function

    m_poseSubscriber = new message_filters::Subscriber<nav_msgs::Odometry>(m_nh, m_odometry, 5);
    m_poseSubscriber->registerCallback(boost::bind(&StageInterface::updateCurrentPosition, this, _1));

    // Register publishers

    m_trajectory_Publisher = m_nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command_trajectory", 1, true);

    m_command_Path_Publisher = m_nh.advertise<nav_msgs::Path>("command_path", 1, true);

    m_reference_Publisher = m_nh.advertise<geometry_msgs::PoseStamped> (m_reference_topic ,1, true);

    // Ros services 
  
    m_exploration_path_client = m_nh.serviceClient<stage_planner::explorationPath>(
      "next_exploration_path");

    m_planner_initialization_srv = m_nh.advertiseService("initialize_planner", &StageInterface::initializePlanner, this);

    m_planner_stop_srv = m_nh.advertiseService ("stop_planner", &StageInterface::stopPlanner, this);

    m_planner_homing_srv = m_nh.advertiseService ("start_homing_planner", &StageInterface::homingPlanner, this);


};


bool StageInterface::initializePlanner (stage_planner::initializePlanning::Request &req,
                                        stage_planner::initializePlanning::Response &res)

{
  
  if (req.initialize) {
    initialization = true;
    explore = true;
    res.success = true;
  }

  return true;

}

bool StageInterface::stopPlanner (stage_planner::stopPlanning::Request &req,
                                        stage_planner::stopPlanning::Response &res)

{
  
  if (req.stop) {
    stop_planning = true;
    explore = false;
    res.success = true;
  }

  return true;

}

bool StageInterface::homingPlanner (stage_planner::homePlanning::Request &req,
                                        stage_planner::homePlanning::Response &res)

{
  
  if (req.homing) {
    return_home = true;
    explore = false;
    res.success = true;
  }

  return true;

}

Eigen::Quaternion<float> StageInterface::Euler2Quaternion(ufo::math::Vector3 v)
{
  float roll = v.x();
  float pitch = v.y();
  float yaw = v.z();

  Eigen::Quaternion<float> q;

  q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  return q;
}

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
  }

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

ufo::math::Vector3 StageInterface::fetchCurrentPosition ()
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

void StageInterface::fetchExplorationPath () {

  stage_planner::explorationPath exploration_path_srv;
  
  if (explore) {

    exploration_path_srv.request.exploration = true;
    exploration_path_srv.request.homing = false; 

  } 
  
  if (return_home and !explore) {
    
    exploration_path_srv.request.exploration = false;
    exploration_path_srv.request.homing = true;

  }

  if (m_exploration_path_client.call(exploration_path_srv)) {
    
    exploration_path_ = exploration_path_srv.response.path;

    planning_status_local = exploration_path_srv.response.planning_status_local;
    planning_status_global = exploration_path_srv.response.planning_status_global;    
    planning_status_homing = exploration_path_srv.response.planning_status_homing; 
    
    // TODO Add a ros_info_cond to log planning status 
  
  }

}

void StageInterface::segmentPath(const nav_msgs::Path& path,
                                 nav_msgs::Path& path_seg) 
{
  path_seg.poses.clear();
  double v_max_ = 0.5;
  double yaw_rate_max_ = 0.5;
  if (path.poses.size() == 0) return;
  if (path.poses.size() == 1) path_seg.poses.push_back(path.poses[0]);

  for (int i = 0; i < (path.poses.size() - 1); ++i) {
    // Interpolate each segment.
    Eigen::Vector3d start(path.poses[i].pose.position.x, path.poses[i].pose.position.y,
                          path.poses[i].pose.position.z);
    Eigen::Vector3d end(path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y,
                        path.poses[i + 1].pose.position.z);
    Eigen::Vector3d distance = end - start;
    double yaw_start = tf::getYaw(path.poses[i].pose.orientation);
    double yaw_end = tf::getYaw(path.poses[i + 1].pose.orientation);
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
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position = pose.position;
        pose_stamped.pose.orientation = pose.orientation;
        path_seg.poses.push_back(pose_stamped);
      }
    }
  }
} // segmentPath

void StageInterface::generateTrajectory() {

    nav_msgs::Path raw_path;  

    if (initialization) {

    nav_msgs::Path initialization_path;  
    std::vector<geometry_msgs::PoseStamped> initialization_poses;  
    geometry_msgs::PoseStamped init_pose;  
    
    init_pose.pose.position.x = fetchCurrentPosition().x() + 2;
    init_pose.pose.position.y = fetchCurrentPosition().y();
    init_pose.pose.position.z = fetchCurrentPosition().z() + 1;
    init_pose.pose.orientation.x = 0;
    init_pose.pose.orientation.y = 0;
    init_pose.pose.orientation.z = 0;
    init_pose.pose.orientation.w = 1;
    initialization_path.poses.push_back(init_pose);
    raw_path = initialization_path;
    initialization = false;
  
  } else {

    raw_path = exploration_path_;
  }
  
  // TODO Insert current position at the begining of exploration path to get smooth start of next path following action.

    nav_msgs::Path new_path;
    new_path.poses.clear();
    segmentPath(raw_path, new_path);

    n_seq_++;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_array_;
    mav_msgs::EigenTrajectoryPoint trajectory_point_;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg_;
    std::vector<geometry_msgs::Pose> executing_path_;
    
    trajectory_array_.header.seq = n_seq_;
    trajectory_array_.header.stamp = ros::Time::now();
    trajectory_array_.header.frame_id = "world";
    trajectory_array_.points.clear();
    double time_sum = 0;
    
    command_path.poses.clear();
    command_path.header.frame_id = "world";

    for (int i = 0; i < new_path.poses.size(); i++) {
      
      geometry_msgs::PoseStamped pose_ref;

      double yaw = tf::getYaw(new_path.poses[i].pose.orientation);
      Eigen::Vector3d p(new_path.poses[i].pose.position.x, new_path.poses[i].pose.position.y,
                        new_path.poses[i].pose.position.z);
      trajectory_point_.position_W.x() = p.x();
      trajectory_point_.position_W.y() = p.y();
      trajectory_point_.position_W.z() = p.z();
      trajectory_point_.setFromYaw(yaw);
      pose_ref = new_path.poses[i];
      
      mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(
          trajectory_point_, &trajectory_point_msg_);
      time_sum += 2*dt_;
      trajectory_point_msg_.time_from_start = ros::Duration(time_sum);
      // trajectory_array_.points.insert(trajectory_array_.points.begin(),trajectory_point_msg_);
      trajectory_array_.points.push_back(trajectory_point_msg_);
      // geometry_msgs::Pose command_pose;
      // command_pose.position = new_path.poses[i].pose.position;
      // command_pose.orientation = new_path.poses[i].pose.orientation;
      
      // TODO Need to optimize this insertion! may be assign new_path directly to command path?

      command_path.poses.push_back(new_path.poses[i]);

    }
    m_trajectory_Publisher.publish(trajectory_array_);
    m_command_Path_Publisher.publish(command_path);

} // executeCurrentPath

// TODO Add path truncation function to assure safety in case of UAV. 

void StageInterface::followPath () 
{
  
  if (command_path.poses.empty()) {
    ROS_WARN ("Command path is empty ");
  }

  for (int i = 0; i < command_path.poses.size(); ) {

    ufo::math::Vector3 way_point;
    way_point.x() = command_path.poses[i].pose.position.x;
    way_point.y() = command_path.poses[i].pose.position.y; 
    way_point.z() = command_path.poses[i].pose.position.z;

    m_reference_Publisher.publish(command_path.poses[i]);
    
    ROS_INFO_STREAM ("Following reference : " << way_point.x() << " , " << way_point.y() << " , " << way_point.z() );
    
    bool next_wp = true;

    while (next_wp) {
      
      double delta = calculateDistance (way_point, fetchCurrentPosition());
      
      if (delta < 0.5) {
        next_wp = false;
      }
    }

    i++;
  }
}

bool StageInterface::reachedEndOfPath () {
  
  ufo::math::Vector3 end_of_path; 
  end_of_path.x() = command_path.poses.back().pose.position.x;
  end_of_path.y() = command_path.poses.back().pose.position.y;
  end_of_path.z() = command_path.poses.back().pose.position.z;
  
  double d = calculateDistance(fetchCurrentPosition(), end_of_path);

  if (d < 1) {
    return true;
  } else {
    return false;
  }

}

void StageInterface::run_planner () 
{
 
  ros::Rate rr(10);
  bool run = true;
  
  while (run) {
    
    
    
    run = ros::ok();
    ros::spinOnce();
    rr.sleep();

  }

  ROS_WARN("Interrupted ....");

}

