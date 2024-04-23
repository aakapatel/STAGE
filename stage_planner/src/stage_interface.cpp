#include "stage_interface.h"

#include <cstdlib>

#include "std_srvs/Empty.h"
using namespace std::chrono;

StageInterface::StageInterface(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
        map_manager)
    : nh_(nh),
      nh_private_(nh_private),
      map_manager_(map_manager),
      ac_("stage_interface_path", true)

{
  StageInterface::initializeAttributes();
};

void StageInterface::initializeAttributes() {
  odometry_Subscriber_ =
      nh_.subscribe("odometry", 100, &StageInterface::odometryCallback, this);

  trajectory_Publisher =
      nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          "command/trajectory", 1, true);

  command_Path_Publisher =
      nh_.advertise<nav_msgs::Path>("command_path", 1, true);

  reference_Publisher =
      nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1, true);

  // Ros services

  planner_takeoff_service_ = nh_.serviceClient<std_srvs::Empty>("takeoff");

  planner_land_service_ = nh_.serviceClient<std_srvs::Empty>("land");

  planner_initialization_service_ = nh_.advertiseService(
      "initialize_planner", &StageInterface::initializePlanner, this);

  planner_stop_service_ =
      nh_.advertiseService("stop_planner", &StageInterface::stopPlanner, this);

  planner_homing_service_ = nh_.advertiseService(
      "start_homing_planner", &StageInterface::homingPlanner, this);

  // Planner periodic timer

  planner_periodic_timer_ = nh_.createTimer(
      ros::Duration(0.1), &StageInterface::periodicTimerCallback, this);

  graph_manager_ = new GraphManager(nh_, nh_private_, map_manager_);
  visualization_server_ = new VisualizationServer(nh_, nh_private_);

  stat_.reset(new SampleStatistic());
  stat_chrono_.reset(new SampleStatistic());
  stat_->init(robotState_);
}

void StageInterface::loadParams() {
  std::string ns = ros::this_node::getName();
  sensor_params_.loadParams(ns + "/SensorParams");
  sensor_name = sensor_params_.sensor_list[0];
  robot_params_.loadParams(ns + "/RobotParams");
  global_space_params_.loadParams(ns + "/BoundedSpaceParams/Global");
  local_space_params_.loadParams(ns + "/BoundedSpaceParams/Local");
  planning_params_.loadParams(ns + "/PlanningParams");
  robot_box_size_ = robot_params_.size;
  map_manager_->setRobotRadius(robot_box_size_.norm() / 2.0);
}

void StageInterface::setSharedParams(
    const RobotParams& robot_params,
    const BoundedSpaceParams& global_space_params,
    const BoundedSpaceParams& local_space_params) {
  StageInterface::loadParams();
  graph_manager_->setSharedParams(robot_params, global_space_params,
                                  local_space_params);
  init_x = planning_params_.init_forward_x;
  dt_ = planning_params_.dt;
  v_max_ = planning_params_.v_max;
  yaw_rate_max_ = planning_params_.yaw_rate_max;
  world_frame_id = planning_params_.global_frame_id;
}

double StageInterface::projectSample(Eigen::Vector3d& sample,
                                     MapManager::VoxelStatus& voxel_status) {
  double max_proj_len = 5.0;

  float voxel_size = map_manager_->getResolution();

  int unknown_count = 0;
  double central_ray_len = 0.0;
  std::vector<Eigen::Vector3d> extra_samples(5, Eigen::Vector3d::Zero());
  extra_samples[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
  extra_samples[1] = Eigen::Vector3d(0.5, 0.0, 0.5);
  extra_samples[2] = Eigen::Vector3d(-0.5, 0.0, 0.5);
  extra_samples[3] = Eigen::Vector3d(0.0, 0.5, 0.5);
  extra_samples[4] = Eigen::Vector3d(0.0, -0.5, 0.5);
  for (int i = 0; i < extra_samples.size(); ++i) {
    Eigen::Vector3d delta = extra_samples[i];
    Eigen::Vector3d start = sample + delta;
    Eigen::Vector3d end = start - Eigen::Vector3d(0.0, 0.0, max_proj_len);

    if (planning_params_.interpolate_projection_distance) {
      double ground_dist = 0.0;
      Eigen::Vector3d point = start;
      while (ground_dist < max_proj_len) {
        double tsdf_dist = map_manager_->getPointDistance(point);
        if (tsdf_dist < 0.0) {
          // Hit Unknown
          if (i == 0) {
            central_ray_len = (point - start).norm();
          }
          ++unknown_count;
          break;
        } else {
          if (tsdf_dist <= voxel_size) {
            // Hit occupied
            voxel_status = MapManager::VoxelStatus::kOccupied;
            sample(0) = start(0);
            sample(1) = start(1);
            return ground_dist + tsdf_dist;
          } else {
            // Still in free
            ground_dist += tsdf_dist;
            point -= Eigen::Vector3d(0.0, 0.0, tsdf_dist - 0.02);
          }
        }
      }
    } else {
      Eigen::Vector3d end_voxel;
      double tsdf_dist;
      MapManager::VoxelStatus vs =
          map_manager_->getRayStatus(start, end, true, end_voxel, tsdf_dist);

      if (vs == MapManager::VoxelStatus::kOccupied) {
        double ray_len = std::abs(start(2) - end_voxel(2));
        if (i == 0) {
          central_ray_len = ray_len;
        }
        voxel_status = MapManager::VoxelStatus::kOccupied;
        sample(0) = start(0);
        sample(1) = start(1);
        return ray_len;
      } else if (vs == MapManager::VoxelStatus::kUnknown) {
        double ray_len = std::abs(start(2) - end_voxel(2));
        if (i == 0) {
          central_ray_len = ray_len;
        }
        ++unknown_count;
      }
    }
  }

  if (unknown_count >= extra_samples.size()) {
    voxel_status = MapManager::VoxelStatus::kUnknown;
    return central_ray_len;
  }

  voxel_status = MapManager::VoxelStatus::kFree;
  return -1.0;
}

void StageInterface::odometryCallback(const nav_msgs::Odometry& odom) {
  odom_received = true;

  robotState_[0] = odom.pose.pose.position.x;
  robotState_[1] = odom.pose.pose.position.y;
  robotState_[2] = odom.pose.pose.position.z;
  robotState_[3] = tf::getYaw(odom.pose.pose.orientation);

  if (robot_params_.type == RobotType::kGroundRobot) {
    MapManager::VoxelStatus vs;
    Eigen::Vector3d root_pos = robotState_.head(3);
    double ground_height = projectSample(root_pos, vs);
    robotState_[2] += (planning_params_.max_ground_height - ground_height);
  }
  if (planner_initialized) {
    graph_manager_->setState(robotState_);
    // double time_elapsed = 0.0;
    if ((ros::Time::now()).toSec() != 0.0) {
      if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
      time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
    }
    time_budget_remaining_ =
        planning_params_.time_budget_before_landing - time_elapsed;
  }

  if ((time_budget_remaining_ <= 0 || manual_homing_triggered_) &&
      !homing_planner_triggered_) {
    // ROS_WARN("Homing should be engaged \n");
    exploration_path_.poses.clear();
    planner_status_ == StageInterface::PlannerStatus::HOMING;
    homing_planner_triggered_ = true;
  }

  if (reachedEndOfPath()) {
    path_execution_complete_ = true;
  } else {
    path_execution_complete_ = false;
  }
}

bool StageInterface::initializePlanner(
    stage_planner::initializePlanning::Request& req,
    stage_planner::initializePlanning::Response& res) {
  if (!odom_received) {
    return false;
  }

  StageInterface::setSharedParams(robot_params_, global_space_params_,
                                  local_space_params_);

  std::string namespace_str;
  std::string command;
  std_srvs::Empty takeoff_srv;
  if (req.takeoff) {
    switch (planning_params_.planner_mode) {
      case PlannerMode::kSim:

        ROS_WARN("PlannerMode : kSim ! \n");
        if (planner_takeoff_service_.call(takeoff_srv)) {
          ROS_WARN("Takeoff service sent! \n");
        } else {
          // ROS_WARN("Takeoff service failed \n");
        }
        break;
      case PlannerMode::kReal:
        ROS_WARN("PlannerMode : kReal ! \n");
        namespace_str = ros::this_node::getNamespace();
        // send Arm command
        command =
            "rosservice call " + namespace_str + "/mavros/cmd/arming \"True\"";
        (void)system(command.c_str());
        // Wait for 2 seconds
        ros::Duration(2).sleep();

        // set offboard mode
        command = "rosservice call " + namespace_str +
                  "/mavros/set_mode \"base_mode: 0 "
                  "custom_mode: 'OFFBOARD'\"";
        (void)system(command.c_str());
        // Wait for 2 seconds
        ros::Duration(2).sleep();

        // Publish start string to controller
        command = "rosservice call " + namespace_str +
                  "/start std_msgs/String \"data: ''\" -1";
        (void)system(command.c_str());
        break;
      default:
        ROS_ERROR("Planner mode not set in params \n");
        break;
    }

    res.mission_duration = planning_params_.time_budget_before_landing;
  }

  if (req.initialize) {
    ROS_INFO("Planner initialization requested \n");

    ROS_WARN_STREAM("Init robotState Z : " << robotState_[2] << "\n");

    graph_manager_->loadParams();
    graph_manager_->reset();
    graph_manager_->setSharedParams(robot_params_, global_space_params_,
                                    local_space_params_);
    graph_manager_->setGlobalFrame(planning_params_.global_frame_id);
    homing_planner_triggered_ = false;  // safety guard

    // clear and initialize first init motion path
    exploration_path_.poses.clear();
    path_iterator_ = 0;
    rostime_start_ = ros::Time::now();
    nav_msgs::Path initialization_path;
    initialization_path.poses.clear();
    std::vector<geometry_msgs::PoseStamped> initialization_poses;
    initialization_poses.clear();
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
    if (robot_params_.type == RobotType::kGroundRobot) {
      ROS_WARN("robot type --> ground \n");
      init_pose.pose.position.z = robotState_[2];
    } else {
      ROS_WARN("robot type --> aerial \n");
      init_pose.pose.position.z = robotState_[2];
    }
    init_pose.pose.orientation.x = 0;
    init_pose.pose.orientation.y = 0;
    init_pose.pose.orientation.z = 0;
    init_pose.pose.orientation.w = 1;
    initialization_path.poses.push_back(init_pose);
    init_pose.pose.position.x = robotState_[0] + init_x;
    init_pose.pose.position.y = robotState_[1];
    if (robot_params_.type == RobotType::kGroundRobot) {
      init_pose.pose.position.z = robotState_[2];
    } else {
      init_pose.pose.position.z = robotState_[2];
    }
    init_pose.pose.orientation.x = 0;
    init_pose.pose.orientation.y = 0;
    init_pose.pose.orientation.z = 0;
    init_pose.pose.orientation.w = 1;
    initialization_path.poses.push_back(init_pose);

    StageInterface::generateTrajectory(initialization_path);
    command_Path_Publisher.publish(command_path);
    planner_initialized = true;
    StageInterface::callActionServer();
    planner_status_ = StageInterface::PlannerStatus::READY;

    res.success = true;
    ROS_INFO("Initialization Successful : Ready ");

    res.mission_duration = planning_params_.time_budget_before_landing;
  }

  return true;
}

bool StageInterface::stopPlanner(stage_planner::stopPlanning::Request& req,
                                 stage_planner::stopPlanning::Response& res)

{
  std::string namespace_str;
  std::string command;
  std_srvs::Empty land_srv;
  if (req.land) {
    switch (planning_params_.planner_mode) {
      case PlannerMode::kSim:
        if (planner_land_service_.call(land_srv)) {
          ROS_WARN("Land service sent! \n");
        } else {
          // ROS_WARN("Takeoff service failed \n");
        }
        break;
      case PlannerMode::kReal:
        namespace_str = ros::this_node::getNamespace();

        // Publish safety land string to controller
        command = "rosservice call " + namespace_str +
                  "/safety_land std_msgs/String \"data: ''\" -1";
        (void)system(command.c_str());

        ros::Duration(4).sleep();

        // set manual mode
        command = "rosservice call " + namespace_str +
                  "/mavros/set_mode \"base_mode: 0 "
                  "custom_mode: 'MANUAL'\"";
        (void)system(command.c_str());
        // Wait for 2 seconds
        ros::Duration(3).sleep();
        // send Disarm command
        command =
            "rosservice call " + namespace_str + "/mavros/cmd/arming \"False\"";
        (void)system(command.c_str());

        break;
      default:
        ROS_ERROR("Planner mode not set in params \n");
        break;
    }
  }

  if (req.stop) {
    planner_status_ = StageInterface::PlannerStatus::NOT_READY;
  }
  ROS_INFO("Stop Success ^_^");
  return true;
}

bool StageInterface::homingPlanner(stage_planner::homePlanning::Request& req,
                                   stage_planner::homePlanning::Response& res)

{
  if (req.homing) {
    manual_homing_triggered_ = true;
    homing_planner_triggered_ = true;
    return_home = true;
    explore = false;
    res.success = true;
  }
  // ROS_WARN("Return to home requested ");
  ROS_WARN("Return to home requested ");
  return true;
}

Eigen::Vector3d StageInterface::quaternion_to_rpy(Eigen::Quaterniond q) {
  const double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  const double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
  const double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

  return Eigen::Vector3d{roll, pitch, yaw};
}

Eigen::Quaternion<float> StageInterface::Euler2Quaternion(Eigen::Vector3d v) {
  float roll = v.x();
  float pitch = v.y();
  float yaw = v.z();

  Eigen::Quaternion<float> q;

  q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

  return q;
}

double StageInterface::calculateDistance(Eigen::Vector3d point1,
                                         Eigen::Vector3d point2) {
  double d;

  if (robot_params_.type == RobotType::kGroundRobot) {
    d = sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2));
  } else {
    d = sqrt(pow(point1.x() - point2.x(), 2) + pow(point1.y() - point2.y(), 2) +
             pow(point1.z() - point2.z(), 2));
  }

  return d;
}

void StageInterface::actionActiveCallback() {
  ROS_INFO("Action Server : Goal went active");
}

void StageInterface::actionFeedbackCallback(
    const stage_planner::pathFollowerActionFeedbackConstPtr& feedback) {
  // leavingempty for now, can do something here based on path follow feedback
}

bool StageInterface::actionDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const stage_planner::pathFollowerActionResultConstPtr& result) {
  ROS_INFO("Finished follow action.");
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    // trigger replanning here ?

  }

  else {
    // can have action server error output here
  }
  return true;
}

void StageInterface::segmentPath(const nav_msgs::Path& path,
                                 nav_msgs::Path& path_seg) {
  path_seg.poses.clear();
  if (path.poses.size() == 0) return;
  if (path.poses.size() == 1) path_seg.poses.push_back(path.poses[0]);

  for (int i = 0; i < (path.poses.size() - 1); ++i) {
    // Interpolate each segment.
    Eigen::Vector3d start(path.poses[i].pose.position.x,
                          path.poses[i].pose.position.y,
                          path.poses[i].pose.position.z);
    Eigen::Vector3d end(path.poses[i + 1].pose.position.x,
                        path.poses[i + 1].pose.position.y,
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
}  // segmentPath

void StageInterface::generateTrajectory(nav_msgs::Path raw_path) {
  nav_msgs::Path new_path;
  new_path.poses.clear();
  StageInterface::segmentPath(raw_path, new_path);
  // new_path = raw_path;
  n_seq_++;
  trajectory_msgs::MultiDOFJointTrajectory trajectory_array_;
  mav_msgs::EigenTrajectoryPoint trajectory_point_;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg_;
  std::vector<geometry_msgs::Pose> executing_path_;

  // set actions server goal

  goal_path.clear();

  trajectory_array_.header.seq = n_seq_;
  trajectory_array_.header.stamp = ros::Time::now();
  trajectory_array_.header.frame_id = world_frame_id;
  trajectory_array_.points.clear();
  double time_sum = 0;

  geometry_msgs::PoseStamped pose_ref;
  command_path.poses.clear();
  pose_ref.header.frame_id = world_frame_id;
  pose_ref.header.stamp = ros::Time::now();
  command_path.header.frame_id = world_frame_id;
  command_path.header.stamp = ros::Time::now();

  for (int i = 0; i < new_path.poses.size(); i++) {
    double yaw = tf::getYaw(new_path.poses[i].pose.orientation);
    Eigen::Vector3d p(new_path.poses[i].pose.position.x,
                      new_path.poses[i].pose.position.y,
                      new_path.poses[i].pose.position.z);
    trajectory_point_.position_W.x() = p.x();
    trajectory_point_.position_W.y() = p.y();
    trajectory_point_.position_W.z() = p.z();
    trajectory_point_.setFromYaw(yaw);
    pose_ref = new_path.poses[i];

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point_,
                                                       &trajectory_point_msg_);
    time_sum += dt_;
    trajectory_point_msg_.time_from_start = ros::Duration(time_sum);
    trajectory_array_.points.push_back(trajectory_point_msg_);

    // TODO Need to optimize this insertion! may be assign new_path directly to
    // command path?

    command_path.poses.push_back(pose_ref);
    goal_path.push_back(pose_ref);
  }
  
  // set the goal path for action server --> Ground robot
  goal.path = goal_path;
  // Publish the trajectory --> Aerial robot 
  // trajectory_Publisher.publish(trajectory_array_);

}  // generateTrajectory

// TODO Add path truncation function to assure safety in case of UAV.

bool StageInterface::reachedEndOfPath() {
  if (command_path.poses.empty()) {
    // ROS_INFO("command path empty \n");
    return false;
  }

  Eigen::Vector3d end_of_path;
  end_of_path.x() = command_path.poses.back().pose.position.x;
  end_of_path.y() = command_path.poses.back().pose.position.y;
  end_of_path.z() = command_path.poses.back().pose.position.z;

  double d = calculateDistance(robotState_.head(3), end_of_path);

  if (d < planning_params_.distance_to_new_path) {
    return true;
  } else {
    return false;
  }
}

void StageInterface::callActionServer() {
  if (robot_params_.type == RobotType::kGroundRobot) {
    ROS_INFO("Ready to set goal to Action Server \n");
    ac_.waitForServer();
    ac_.sendGoal(
        goal, boost::bind(&StageInterface::actionDoneCallback, this, _1, _2),
        boost::bind(&StageInterface::actionActiveCallback, this),
        boost::bind(&StageInterface::actionFeedbackCallback, this, _1));
  }
}

void StageInterface::periodicTimerCallback(const ros::TimerEvent& event) {
  if (!manual_homing_triggered_ && !path_execution_complete_) {
    return;
  } else if (!path_execution_complete_ && manual_homing_triggered_) {
    planner_status_ = StageInterface::PlannerStatus::HOMING;
    manual_homing_triggered_ = false;
  }

  // ROS_INFO_STREAM("Time remaining : " << time_budget_remaining_ << "s \n");

  high_resolution_clock::time_point start_total = high_resolution_clock::now();
  switch (planner_status_) {
    case StageInterface::PlannerStatus::READY:
      ROS_INFO("[PLANNER STATUS : ] Ready for new path \n");
      exploration_path_.poses.clear();
      path_iterator_ = 0;

      graph_status_ = graph_manager_->runLocalPlanner();
      switch (graph_status_) {
        case GraphManager::GraphStatus::OK:
          ROS_INFO("[LOCAL GRAPH STATUS] OK \n");
          exploration_path_ = graph_manager_->getLocalExplorationPath();
          break;
        case GraphManager::GraphStatus::ERR_KDTREE:
          ROS_WARN("[LOCAL GRAPH STATUS] Issue occurred with kdtree data. \n");
          break;
        case GraphManager::GraphStatus::ERR_NO_FEASIBLE_PATH:
          ROS_WARN("[LOCAL GRAPH STATUS] No feasible path was found. \n");
          break;
        case GraphManager::GraphStatus::NO_GAIN:
          ROS_WARN("[LOCAL GRAPH STATUS] Not enough gain \n");
          break;
        case GraphManager::GraphStatus::NOT_OK:
          ROS_WARN("[LOCAL GRAPH STATUS] NOT OK \n");
          break;
        default:
          ROS_WARN(
              "[LOCAL GRAPH ERROR] Error occurred in building local graph \n");
          break;
      }

      if (graph_status_ == GraphManager::GraphStatus::NO_GAIN) {
        graph_status_ = graph_manager_->runGlobalPlanner();
        switch (graph_status_) {
          case GraphManager::GraphStatus::OK:
            ROS_INFO("[GLOBAL GRAPH STATUS] OK \n");
            exploration_path_ = graph_manager_->getGlobalExplorationPath();
            break;
          case GraphManager::GraphStatus::NO_GAIN:
            ROS_WARN("[GLOBAL GRAPH STATUS] Not enough gain \n");
            break;
          case GraphManager::GraphStatus::NOT_OK:
            ROS_WARN("[GLOBAL GRAPH STATUS] NOT OK \n");
            break;
          default:
            ROS_WARN("[GLOBAL GRAPH STATUS] Error occurred in global graph.");
            break;
        }
      }

      if (!exploration_path_.poses.empty()) {
        path_iterator_ = 0;
        planner_status_ = StageInterface::PlannerStatus::EXECUTING_PATH;
      }
      break;

    case StageInterface::PlannerStatus::EXECUTING_PATH:
      ROS_INFO("[PLANNER STATUS : ] Executing path \n");

      if (path_iterator_ < (exploration_path_.poses.size())) {
        temp_path.poses.clear();
        geometry_msgs::PoseStamped p;
        if (path_iterator_ == 0) {
          p.pose.position.x = robotState_[0];
          p.pose.position.y = robotState_[1];
          p.pose.position.z = robotState_[2];
          p.pose.orientation =
              exploration_path_.poses[path_iterator_].pose.orientation;

        } else {
          p.pose.position.x =
              exploration_path_.poses[path_iterator_ - 1].pose.position.x;
          p.pose.position.y =
              exploration_path_.poses[path_iterator_ - 1].pose.position.y;
          p.pose.position.z =
              exploration_path_.poses[path_iterator_ - 1].pose.position.z;
          p.pose.orientation =
              exploration_path_.poses[path_iterator_ - 1].pose.orientation;
        }
        p.header.stamp = ros::Time::now();
        p.header.frame_id = world_frame_id;
        geometry_msgs::PoseStamped p1 = exploration_path_.poses[path_iterator_];
        Eigen::Vector3d v(p.pose.position.x, p.pose.position.y,
                          p.pose.position.z);
        Eigen::Vector3d v1(p1.pose.position.x, p1.pose.position.y,
                           p1.pose.position.z);
        MapManager::VoxelStatus vs =
            map_manager_->getPathStatus(v, v1, robot_box_size_, true);
        if (MapManager::VoxelStatus::kOccupied != vs) {
          temp_path.poses.push_back(p);
          temp_path.poses.push_back(p1);
          path_iterator_ = path_iterator_ + 1;
          planner_status_ = StageInterface::PlannerStatus::EXECUTING_PATH;
          StageInterface::generateTrajectory(temp_path);
          if (!command_path.poses.empty()) {
            command_Path_Publisher.publish(command_path);
            StageInterface::callActionServer();  // only for ground robot
          }
        } else {
          ROS_ERROR("Path segment not traversable \n");
          path_iterator_ = 0;
          exploration_path_.poses.clear();
          if (homing_planner_triggered_) {
            planner_status_ = StageInterface::PlannerStatus::HOMING;
          } else {
            planner_status_ = StageInterface::PlannerStatus::READY;
          }
        }
      } else {
        path_iterator_ = 0;
        exploration_path_.poses.clear();

        if (homing_planner_triggered_) {
          planner_status_ = StageInterface::PlannerStatus::HOMING;
        } else {
          planner_status_ = StageInterface::PlannerStatus::READY;
        }
      }
      break;

    case StageInterface::PlannerStatus::HOMING:
      ROS_WARN("Auto homing triggered \n");
      exploration_path_.poses.clear();
      exploration_path_ = graph_manager_->getHomingPath();
      if (!exploration_path_.poses.empty()) {
        path_iterator_ = 0;
        planner_status_ = StageInterface::PlannerStatus::EXECUTING_HOMING_PATH;
      }
      break;

    case StageInterface::PlannerStatus::EXECUTING_HOMING_PATH:
      ROS_INFO("[PLANNER STATUS : ] Executing homing path \n");

      if (path_iterator_ < (exploration_path_.poses.size())) {
        temp_path.poses.clear();
        geometry_msgs::PoseStamped p;
        p.pose.position.x = robotState_[0];
        p.pose.position.y = robotState_[1];
        p.pose.position.z = robotState_[2];
        p.pose.orientation =
            exploration_path_.poses[path_iterator_].pose.orientation;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = world_frame_id;
        temp_path.poses.push_back(p);
        temp_path.poses.push_back(exploration_path_.poses[path_iterator_]);
        path_iterator_ = path_iterator_ + 1;
        planner_status_ = StageInterface::PlannerStatus::EXECUTING_HOMING_PATH;
        StageInterface::generateTrajectory(temp_path);
        if (!command_path.poses.empty()) {
          command_Path_Publisher.publish(command_path);
          StageInterface::callActionServer();  // only for ground robot
        }
      } else {
        path_iterator_ = 0;
        exploration_path_.poses.clear();
        if (homing_planner_triggered_) {
          planner_status_ = StageInterface::PlannerStatus::NOT_READY;
        }
      }
      break;

    case StageInterface::PlannerStatus::NOT_READY:
      ROS_WARN("[PLANNER STATUS] NOT READY \n");
      break;

    default:
      break;
  }

  high_resolution_clock::time_point stop_total = high_resolution_clock::now();

  auto duration_total =
      duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  if (graph_status_ == GraphManager::GraphStatus::OK &&
      duration_total.count() > 10) {
    ++planning_iteration_;
    ROS_INFO_STREAM("\033[;34m[Planning iter -> "
                    << planning_iteration_ << "] \t Exec time :"
                    << duration_total.count() << " ms \033[0m\n");
  }
}
