#include "stage_interface.h"

using namespace std;

StageInterface::StageInterface(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
  : nh_(nh),
  nh_private_(nh_private),
  m_odometry("/hummingbird/ground_truth/odometry"),
  m_reference_topic("/hummingbird/command/pose"),
  init_x_(1),
  init_y_(1),
  init_z_(1),
  distance_to_new_path_(1),
  robot_type_("kAerial")

{

  // Set variables
  nh_private_.param("odometry_topic", m_odometry, m_odometry);
  nh_private_.param("reference_topic", m_reference_topic, m_reference_topic);

  nh_private_.param("robot_type", robot_type_, robot_type_);
  nh_private_.param("init_x", init_x_, init_x_);
  nh_private_.param("init_y", init_y_, init_y_);
  nh_private_.param("init_z", init_z_, init_z_);

  nh_private_.param("distance_to_new_path", distance_to_new_path_, distance_to_new_path_);

  // Register to the subscriber topic and register it to the callback function

  m_poseSubscriber = nh_.subscribe(m_odometry, 10, &StageInterface::odomCallback, this);
  // Timers

  run_loop_timer_ = nh_.createTimer(ros::Duration(1), &StageInterface::mainLoop, this);

  // Register publishers

  m_trajectory_Publisher = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1, true);

  m_command_Path_Publisher = nh_.advertise<nav_msgs::Path>("command_path", 1, true);

  m_reference_Publisher = nh_.advertise<geometry_msgs::PoseStamped>(m_reference_topic, 1, true);

  // Ros services

  m_exploration_path_client = nh_.serviceClient<stage_planner::explorationPath>(
    "next_exploration_path");

  m_planner_initialization_srv = nh_.advertiseService("initialize_planner", &StageInterface::initializePlanner, this);

  m_planner_stop_srv = nh_.advertiseService("stop_planner", &StageInterface::stopPlanner, this);

  m_planner_homing_srv = nh_.advertiseService("start_homing_planner", &StageInterface::homingPlanner, this);

  if (robot_type_ == "kGround")
  {
    init_z_ = 0;
  }
};

bool StageInterface::initializePlanner(stage_planner::initializePlanning::Request &req,
                                       stage_planner::initializePlanning::Response &res)

{

  if (req.initialize)
  {
    initialization = true;
    explore = true;
    StageInterface::generateTrajectory();

    m_command_Path_Publisher.publish(command_path);

    // if (reachedEndOfPath()) {
    res.success = true;
    ROS_INFO("Initialization Successful : Ready ");
    // }
  }

  return true;
}

bool StageInterface::stopPlanner(stage_planner::stopPlanning::Request &req,
                                 stage_planner::stopPlanning::Response &res)

{

  if (req.stop)
  {
    stop_planning = true;
    explore = false;
    res.success = true;
  }
  ROS_INFO("Stop Success ^_^");
  return true;
}

bool StageInterface::homingPlanner(stage_planner::homePlanning::Request &req,
                                   stage_planner::homePlanning::Response &res)

{

  if (req.homing)
  {
    return_home = true;
    explore = false;
    res.success = true;
  }
  ROS_INFO("Return to home requested ");
  return true;
}

Eigen::Vector3d StageInterface::quaternion_to_rpy(Eigen::Quaterniond q)
{
  const double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  const double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
  const double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

  return Eigen::Vector3d{roll, pitch, yaw};
}

Eigen::Quaternion<float> StageInterface::Euler2Quaternion(Eigen::Vector3d v)
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

void StageInterface::odomCallback(const nav_msgs::Odometry::ConstPtr &pose)
{

  if (firstTime)
  {

    m_prevPose = *pose;
    firstTime = false;
  }
  else
{
    m_currentPose = *pose;

    Eigen::Vector3d newPoint(pose->pose.pose.position.x,
                             pose->pose.pose.position.y,
                             pose->pose.pose.position.z);
    Eigen::Vector3d oldPoint(m_prevPose.pose.pose.position.x,
                             m_prevPose.pose.pose.position.y,
                             m_prevPose.pose.pose.position.z);

    if (calculateDistance(newPoint, oldPoint) > 2)
    {
      m_prevPose = *pose;
    }
  }
  m_currentPosition.x() = m_currentPose.pose.pose.position.x;
  m_currentPosition.y() = m_currentPose.pose.pose.position.y;
  m_currentPosition.z() = m_currentPose.pose.pose.position.z;

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
  robotState_[3] = yaw * (180 / M_PI);
}

Eigen::Vector3d StageInterface::fetchCurrentPosition()
{
  return m_currentPosition;
}

double StageInterface::calculateDistance(Eigen::Vector3d point1,
                                         Eigen::Vector3d point2)
{

  double d;

  if (robot_type_ == "kAerial")
  {

    d = sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) +
             pow(point1.z() - point2.z(), 2));
  }
  else
  {

    d = sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  }

  return d;
}

bool StageInterface::fetchExplorationPath()
{

  bool got_new_path = false;

  stage_planner::explorationPath exploration_path_srv;

  if (explore)
  {

    exploration_path_srv.request.exploration = true;
    exploration_path_srv.request.homing = false;
  }

  if (return_home and !explore)
  {

    exploration_path_srv.request.exploration = false;
    exploration_path_srv.request.homing = true;
  }

  if (m_exploration_path_client.call(exploration_path_srv))
  {

    if (!exploration_path_srv.response.path.poses.empty())
    {
      got_new_path = true;
      exploration_path_ = exploration_path_srv.response.path;
    }
    else
  {
      got_new_path = false;
    }

    planning_status_local = exploration_path_srv.response.planning_status_local;
    planning_status_global = exploration_path_srv.response.planning_status_global;
    planning_status_homing = exploration_path_srv.response.planning_status_homing;

    // TODO Add a ros_info_cond to log planning status
  }

  return got_new_path;
}

void StageInterface::segmentPath(const nav_msgs::Path &path,
                                 nav_msgs::Path &path_seg)
{
  path_seg.poses.clear();
  double v_max_ = 1;
  double yaw_rate_max_ = 0.1;
  if (path.poses.size() == 0)
    return;
  if (path.poses.size() == 1)
    path_seg.poses.push_back(path.poses[0]);

  for (int i = 0; i < (path.poses.size() - 1); ++i)
  {
    // Interpolate each segment.
    Eigen::Vector3d start(path.poses[i].pose.position.x, path.poses[i].pose.position.y,
                          path.poses[i].pose.position.z);
    Eigen::Vector3d end(path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y,
                        path.poses[i + 1].pose.position.z);
    Eigen::Vector3d distance = end - start;
    double yaw_start = tf::getYaw(path.poses[i].pose.orientation);
    double yaw_end = tf::getYaw(path.poses[i + 1].pose.orientation);
    double yaw_direction = yaw_end - yaw_start;
    if (yaw_direction > M_PI)
    {
      yaw_direction -= 2.0 * M_PI;
    }
    if (yaw_direction < -M_PI)
    {
      yaw_direction += 2.0 * M_PI;
    }

    double dist_norm = distance.norm();
    double disc = std::min(dt_ * v_max_ / dist_norm,
                           dt_ * yaw_rate_max_ / abs(yaw_direction));

    bool int_flag = true;

    if (int_flag)
    {
      for (double it = 0.0; it <= 1.0; it += disc)
      {
        tf::Vector3 origin((1.0 - it) * start[0] + it * end[0],
                           (1.0 - it) * start[1] + it * end[1],
                           (1.0 - it) * start[2] + it * end[2]);
        double yaw = yaw_start + yaw_direction * it;
        if (yaw > M_PI)
          yaw -= 2.0 * M_PI;
        if (yaw < -M_PI)
          yaw += 2.0 * M_PI;
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

void StageInterface::generateTrajectory()
{

  nav_msgs::Path raw_path;

  if (initialization)
  {

    nav_msgs::Path initialization_path;
    std::vector<geometry_msgs::PoseStamped> initialization_poses;
    geometry_msgs::PoseStamped init_pose;

    init_pose.pose.position.x = robotState_[0];
    init_pose.pose.position.y = robotState_[1];
    init_pose.pose.position.z = robotState_[2];
    init_pose.pose.orientation.x = 0;
    init_pose.pose.orientation.y = 0;
    init_pose.pose.orientation.z = 0;
    init_pose.pose.orientation.w = 1;
    initialization_path.poses.push_back(init_pose);
    init_pose.pose.position.x = robotState_[0];
    init_pose.pose.position.y = robotState_[1];
    init_pose.pose.position.z = init_z_;
    init_pose.pose.orientation.x = 0;
    init_pose.pose.orientation.y = 0;
    init_pose.pose.orientation.z = 0;
    init_pose.pose.orientation.w = 1;
    initialization_path.poses.push_back(init_pose);
    init_pose.pose.position.x = robotState_[0] + init_x_;
    init_pose.pose.position.y = robotState_[1] + init_y_;
    init_pose.pose.position.z = init_z_;
    init_pose.pose.orientation.x = 0;
    init_pose.pose.orientation.y = 0;
    init_pose.pose.orientation.z = 0;
    init_pose.pose.orientation.w = 1;
    initialization_path.poses.push_back(init_pose);
    raw_path = initialization_path;
    initialization = false;
  }
  else
{

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

  geometry_msgs::PoseStamped pose_ref;
  command_path.poses.clear();
  pose_ref.header.frame_id = "world";
  command_path.header.frame_id = "world";

  for (int i = 0; i < new_path.poses.size(); i++)
  {

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
    time_sum += dt_;
    trajectory_point_msg_.time_from_start = ros::Duration(time_sum);
    // trajectory_array_.points.insert(trajectory_array_.points.begin(),trajectory_point_msg_);
    trajectory_array_.points.push_back(trajectory_point_msg_);

    // TODO Need to optimize this insertion! may be assign new_path directly to command path?

    command_path.poses.push_back(pose_ref);
  }
  m_trajectory_Publisher.publish(trajectory_array_);
  // m_command_Path_Publisher.publish(command_path);

} // generateTrajectory

// TODO Add path truncation function to assure safety in case of UAV.

bool StageInterface::reachedEndOfPath()
{

  Eigen::Vector3d end_of_path;
  end_of_path.x() = command_path.poses.back().pose.position.x;
  end_of_path.y() = command_path.poses.back().pose.position.y;
  end_of_path.z() = command_path.poses.back().pose.position.z;

  double d = calculateDistance(fetchCurrentPosition(), end_of_path);

  if (d < distance_to_new_path_)
  {
    fetch_new_path_ = true;
    return true;
  }
  else
{
    fetch_new_path_ = false;
    return false;
  }
}

void StageInterface::mainLoop(const ros::TimerEvent &event)
{

  if (explore)
  {

    if (reachedEndOfPath() and !stop_planning)
    {

      if (fetchExplorationPath())
      {

        generateTrajectory();

        m_command_Path_Publisher.publish(command_path);
      }
    }
  }
}
