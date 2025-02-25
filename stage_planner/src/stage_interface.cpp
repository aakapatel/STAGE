#include "stage_interface.h"

#include <cstdlib>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/String.h>

#include "std_srvs/Empty.h"
using namespace std::chrono;

StageInterface::StageInterface(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
    MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
        map_manager)
    : nh_(nh),
      nh_private_(nh_private),
      map_manager_(map_manager),
      action_server_(nh_, "/scout_planner/execute_path",
                     boost::bind(&StageInterface::actionGoalCallback, this, _1),
                     false),
      ac_("stage_interface_path", true)

{
  action_server_.start();
  StageInterface::initializeAttributes();
};

void StageInterface::initializeAttributes() {
  odometry_Subscriber_ =
      nh_.subscribe("odometry", 100, &StageInterface::odometryCallback, this);

  goalSubscriber_ = nh_.subscribe("/move_base_simple/goal", 1,
                                  &StageInterface::goalCallback, this);

  trajectory_Publisher =
      nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          "command/trajectory", 1, true);

  command_Path_Publisher =
      nh_.advertise<nav_msgs::Path>("command_path", 1, true);

  reference_Publisher =
      nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1, true);

  controller_Start_Publisher = nh_.advertise<std_msgs::String>("start", 1);

  controller_Land_Publisher = nh_.advertise<std_msgs::String>("safety_land", 1);

  // Ros services

  planner_takeoff_client_ = nh_.serviceClient<std_srvs::Empty>("takeoff");

  planner_land_client_ = nh_.serviceClient<std_srvs::Empty>("land");

  planner_arming_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  planner_set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  planner_initialization_service_ = nh_.advertiseService(
      "initialize_planner", &StageInterface::initializePlanner, this);

  collision_check_service_ = nh_.advertiseService(
      "collision_check", &StageInterface::collisionCheckCallback, this);

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
  manual_mode_params_.loadParams(ns + "/BoundedSpaceParams/LocalSearch");
  planning_params_.loadParams(ns + "/PlanningParams");
  robot_box_size_ = robot_params_.size;
  map_manager_->setRobotRadius(robot_box_size_.norm() / 2.0);
}

void StageInterface::setSharedParams(
    const RobotParams& robot_params,
    const BoundedSpaceParams& global_space_params,
    const BoundedSpaceParams& local_space_params,
    const BoundedSpaceParams& manual_mode_params) {
  StageInterface::loadParams();
  graph_manager_->setSharedParams(robot_params, global_space_params,
                                  local_space_params, manual_mode_params);
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
  }

  if (received_init_location) {  // manual mode initialization
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

bool StageInterface::collisionCheckCallback(
    xflie_msgs::CollisionCheck::Request& req,
    xflie_msgs::CollisionCheck::Response& res) {
  if (!odom_received) {
    ROS_WARN("Odom not received - collision check service failed \n");
    return false;
  }

  geometry_msgs::Point start = req.start;
  geometry_msgs::Point end = req.end;

  Eigen::Vector3d v(start.x, start.y, start.z);
  Eigen::Vector3d v1(end.x, end.y, end.z);
  MapManager::VoxelStatus vs =
      map_manager_->getPathStatus(v, v1, robot_box_size_, true);
  if (MapManager::VoxelStatus::kOccupied != vs) {
    res.collision_status = false;
    std::cout << "Collision check NOT OK \n" << std::endl;
  } else {
    res.collision_status = true;
    std::cout << "Collision check OK \n" << std::endl;
  }

  return true;
}

bool StageInterface::initializePlanner(
    stage_planner::initializePlanning::Request& req,
    stage_planner::initializePlanning::Response& res) {
  if (!odom_received) {
    ROS_WARN("Odom not received \n");
    return false;
  }

  StageInterface::setSharedParams(robot_params_, global_space_params_,
                                  local_space_params_, manual_mode_params_);

  std_srvs::Empty takeoff_srv;
  mavros_msgs::CommandBool arm_cmd;
  mavros_msgs::SetMode offboard_set_mode;
  std_msgs::String start_msg;
  if (req.takeoff) {
    switch (planning_params_.planner_mode) {
      case PlannerMode::kSim:

        ROS_WARN("PlannerMode : kSim ! \n");
        if (planner_takeoff_client_.call(takeoff_srv)) {
          ros::Duration(2).sleep();
          ROS_WARN("Takeoff service sent! \n");
        } else {
          // ROS_WARN("Takeoff service failed \n");
        }
        break;
      case PlannerMode::kReal:
        ROS_WARN("PlannerMode : kReal ! \n");

        arm_cmd.request.value = true;
        if (planner_arming_client_.call(arm_cmd) && arm_cmd.response.success) {
          ROS_INFO("Drone armed \n");
        } else {
          ROS_ERROR("Failed to arm the drone \n");
        }

        // Wait for 2 seconds
        ros::Duration(2).sleep();

        // Set offboard mode
        offboard_set_mode.request.custom_mode = "OFFBOARD";
        if (planner_set_mode_client_.call(offboard_set_mode) &&
            offboard_set_mode.response.mode_sent) {
          ROS_INFO("Offboard mode set \n");
        } else {
          ROS_ERROR("Failed to set offboard mode \n");
        }

        // Wait for 2 seconds
        ros::Duration(2).sleep();

        start_msg.data = "";
        controller_Start_Publisher.publish(start_msg);

        break;
      default:
        ROS_ERROR("Planner mode not set in params \n");
        break;
    }

    res.mission_duration = planning_params_.time_budget_before_landing;
    rostime_start_ = ros::Time::now();
  }

  if (req.initialize) {
    ROS_INFO("Planner initialization requested \n");

    ROS_WARN_STREAM("Init robotState Z : " << robotState_[2] << "\n");

    graph_manager_->loadParams();
    graph_manager_->reset();
    graph_manager_->setSharedParams(robot_params_, global_space_params_,
                                    local_space_params_, manual_mode_params_);
    graph_manager_->setGlobalFrame(planning_params_.global_frame_id);
    homing_planner_triggered_ = false;  // safety guard

    // clear and initialize first init motion path
    exploration_path_.poses.clear();
    path_iterator_ = 0;
    nav_msgs::Path initialization_path;
    initialization_path.poses.clear();

    /*

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

  */

    planner_initialized = true;
    res.success = true;
    ROS_INFO("Initialization Successful : Ready ");

    res.mission_duration = planning_params_.time_budget_before_landing;
  }

  return true;
}

void StageInterface::actionGoalCallback(
    const scout_planner_msgs::scoutExecutePathActionGoalConstPtr& goal_path) {
  path_to_exec_.poses.clear();
  path_to_exec_ = goal_path->path;
  start_execution_ = true;
  float dt = 0.05;
  ros::Rate rate(1.0 / dt);
  global_path_iterator_ = 0;
  command_path.poses.clear();
  re_planning_attempt_ = 0;

  // Only for trajectory generation Debug purposes
  /*
  StageInterface::generateTrajectory(goal_path->path);
  ROS_INFO_STREAM("Scout planner path length : "
                  << goal_path->path.poses.size()
                  << " & Generated trajectory length : "
                  << command_path.poses.size() << "\n");
  */

  while (ros::ok()) {
    // Check if the action is preempted
    if (action_server_.isPreemptRequested()) {
      ROS_WARN("Path Tracker: Path tracking preempted, stopping");

      path_to_exec_.poses.clear();
      command_path.poses.clear();
      global_path_iterator_ = 0;
      action_server_.setPreempted();

      return;
    }

    if (path_to_exec_.poses.empty()) {
      ROS_WARN("Path Tracker: scout planner path is empty, stopping");

      result_.success = false;
      action_server_.setSucceeded(result_);
      global_path_iterator_ = 0;
      command_path.poses.clear();
      return;
    }

    Eigen::Vector3d curr_pos(robotState_[0], robotState_[1], robotState_[2]);
    Eigen::Vector3d final_pos(path_to_exec_.poses.back().pose.position.x,
                              path_to_exec_.poses.back().pose.position.y,
                              path_to_exec_.poses.back().pose.position.z);

    float goal_reached_threshold = 0.5;
    if (calculateDistance(curr_pos, final_pos) < goal_reached_threshold) {
      ROS_INFO("Path Tracker: Reached end of path");
      result_.success = true;
      action_server_.setSucceeded(result_);
      global_path_iterator_ = 0;
      command_path.poses.clear();
      start_execution_ = false;

      return;
    }

    if (reachedEndOfPath()) {
      std::cout << "reached end of command_path \n" << std::endl;

      if (global_path_iterator_ < path_to_exec_.poses.size()) {
        global_temp_path.poses.clear();
        geometry_msgs::PoseStamped p;

        if (global_path_iterator_ == 0) {
          p.pose.position.x = robotState_[0];
          p.pose.position.y = robotState_[1];
          p.pose.position.z = robotState_[2];
          p.pose.orientation =
              path_to_exec_.poses[global_path_iterator_].pose.orientation;

        } else {
          p.pose.position.x =
              path_to_exec_.poses[global_path_iterator_ - 1].pose.position.x;
          p.pose.position.y =
              path_to_exec_.poses[global_path_iterator_ - 1].pose.position.y;
          p.pose.position.z =
              path_to_exec_.poses[global_path_iterator_ - 1].pose.position.z;
          p.pose.orientation =
              path_to_exec_.poses[global_path_iterator_ - 1].pose.orientation;
        }
        geometry_msgs::PoseStamped p1;

        p1.pose.position.x =
            path_to_exec_.poses[global_path_iterator_].pose.position.x;
        p1.pose.position.y =
            path_to_exec_.poses[global_path_iterator_].pose.position.y;
        p1.pose.position.z =
            path_to_exec_.poses[global_path_iterator_].pose.position.z;
        p1.pose.orientation =
            path_to_exec_.poses[global_path_iterator_].pose.orientation;

        p.header.stamp = p1.header.stamp = ros::Time::now();
        p.header.frame_id = p1.header.frame_id = world_frame_id;
        Eigen::Vector3d v(p.pose.position.x, p.pose.position.y,
                          p.pose.position.z);
        Eigen::Vector3d v1(p1.pose.position.x, p1.pose.position.y,
                           p1.pose.position.z);
        MapManager::VoxelStatus vs =
            map_manager_->getPathStatus(v, v1, robot_box_size_, true);
        if (MapManager::VoxelStatus::kOccupied != vs) {
          global_temp_path.poses.push_back(p);
          global_temp_path.poses.push_back(p1);
          global_path_iterator_ = global_path_iterator_ + 1;
          global_temp_path.header.frame_id = world_frame_id;
          global_temp_path.header.stamp = ros::Time::now();
          StageInterface::generateTrajectory(global_temp_path);
          if (!command_path.poses.empty()) {
            command_Path_Publisher.publish(command_path);
          }
        } else {
          ROS_WARN_STREAM("Global route index "
                          << global_path_iterator_ - 1 << " to "
                          << global_path_iterator_ << " is not traversable \n");

          StateVec s = StateVec::Zero();
          s[0] = final_pos[0];
          s[1] = final_pos[1];
          s[2] = final_pos[2];

          global_temp_path.poses.clear();
          bool re_planning_success = false;

          while (global_temp_path.poses.empty() and re_planning_attempt_ < 10) {
            re_planning_attempt_++;
            global_temp_path = graph_manager_->runPlannerkManualMode(s);
            if (!global_temp_path.poses.empty()) {
              ROS_INFO("Successfully planned alternate path \n");
              global_temp_path.header.frame_id = world_frame_id;
              global_temp_path.header.stamp = ros::Time::now();
              StageInterface::generateTrajectory(global_temp_path);
              command_Path_Publisher.publish(command_path);
              global_path_iterator_ = path_to_exec_.poses.size() - 1;
              re_planning_success = true;
              break;
            }
          }
          if (!re_planning_success) {
            ROS_ERROR("No alternate solution found! :( \n");

            result_.success = false;
            std::string path_status = "Next segment is not traversable";
            std::cout << "global path iterator FB : " << global_path_iterator_
                      << std::endl;
            feedback_.collision_segment_index = global_path_iterator_;
            feedback_.status = path_status;
            action_server_.setSucceeded(result_);
            action_server_.publishFeedback(feedback_);
            return;
          }
        }

      } else {
        result_.success = true;
        std::string path_status =
            "Successfully followed the path, Action complete";
        feedback_.collision_segment_index = global_path_iterator_;
        feedback_.status = path_status;
        action_server_.setSucceeded(result_);
        action_server_.publishFeedback(feedback_);
        return;
      }
    }

    // set feedback here

    std::string path_status = "following path successfully ..";
    feedback_.status = path_status;
    action_server_.publishFeedback(feedback_);
    rate.sleep();
    ros::spinOnce();
  }
}

void StageInterface::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
  StateVec s = StateVec::Zero();
  s[0] = goal_msg->pose.position.x;
  s[1] = goal_msg->pose.position.y;
  s[2] = robotState_[2];
  s[3] = robotState_[3];

  received_init_location = true;
  nav_msgs::Path initialization_path;
  initialization_path.poses.clear();
  initialization_path = graph_manager_->runPlannerkManualMode(s);

  if (initialization_path.poses.empty()) {
    ROS_ERROR("Init path empty \n");
    return;
  }

  StageInterface::generateTrajectory(initialization_path);
  command_Path_Publisher.publish(command_path);
  // received_init_location = true;
  StageInterface::callActionServer();
  planner_status_ = StageInterface::PlannerStatus::READY;
}

bool StageInterface::stopPlanner(stage_planner::stopPlanning::Request& req,
                                 stage_planner::stopPlanning::Response& res)

{
  std_srvs::Empty land_srv;
  std_msgs::String safety_land_msg;
  mavros_msgs::SetMode manual_set_mode;
  mavros_msgs::CommandBool disarm_cmd;
  if (req.land) {
    switch (planning_params_.planner_mode) {
      case PlannerMode::kSim:
        if (planner_land_client_.call(land_srv)) {
          ROS_WARN("Land service sent! \n");
        } else {
          // ROS_WARN("Takeoff service failed \n");
        }
        break;
      case PlannerMode::kReal:

        safety_land_msg.data = "";
        controller_Land_Publisher.publish(safety_land_msg);

        // Wait for 4 seconds
        ros::Duration(4).sleep();

        // Set manual mode
        manual_set_mode.request.custom_mode = "MANUAL";
        if (planner_set_mode_client_.call(manual_set_mode) &&
            manual_set_mode.response.mode_sent) {
          ROS_INFO("Manual mode set \n");
        } else {
          ROS_ERROR("Failed to set manual mode \n");
        }

        // Wait for 3 seconds
        ros::Duration(3).sleep();

        // Send Disarm command
        disarm_cmd.request.value = false;
        if (planner_arming_client_.call(disarm_cmd) &&
            disarm_cmd.response.success) {
          ROS_INFO("Drone disarmed \n");
        } else {
          ROS_ERROR("Failed to disarm the drone \n");
        }

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
    planner_status_ = StageInterface::PlannerStatus::HOMING;
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
  // ROS_INFO("Action Server : Goal went active");
}

void StageInterface::actionFeedbackCallback(
    const stage_planner::pathFollowerActionFeedbackConstPtr& feedback) {
  // leavingempty for now, can do something here based on path follow feedback
}

bool StageInterface::actionDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const stage_planner::pathFollowerActionResultConstPtr& result) {
  // ROS_INFO("Finished follow action.");
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
  if (planning_params_.planner_mode == PlannerMode::kSim) {
    StageInterface::segmentPath(raw_path, new_path);
  } else {
    new_path = raw_path;
  }
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

  // commenting out for xFlie implementation. Only for Test and Debug purposes.

  /*
  if (planning_params_.planner_mode == PlannerMode::kSim) {
    trajectory_Publisher.publish(trajectory_array_);
  }
  */

}  // generateTrajectory

// TODO Add path truncation function to assure safety in case of UAV.

bool StageInterface::reachedEndOfPath() {
  if (command_path.poses.empty()) {
    // ROS_INFO("command path empty \n");
    // return false;
    return true;
  }

  Eigen::Vector3d end_of_path;
  end_of_path.x() = command_path.poses.back().pose.position.x;
  end_of_path.y() = command_path.poses.back().pose.position.y;
  end_of_path.z() = command_path.poses.back().pose.position.z;

  double d = calculateDistance(robotState_.head(3), end_of_path);

  // std::cout << "d : " << d << std::endl;

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
  // if (!manual_homing_triggered_ && !path_execution_complete_) {
  //   return;
  // } else if (!path_execution_complete_ && manual_homing_triggered_) {
  //   planner_status_ = StageInterface::PlannerStatus::HOMING;
  //   manual_homing_triggered_ = false;
  // }
  //
  // ROS_INFO_STREAM("Time remaining : " << time_budget_remaining_ << "s \n");
  //
  // high_resolution_clock::time_point start_total =
  // high_resolution_clock::now(); switch (planner_status_) {
  //   case StageInterface::PlannerStatus::READY:
  //     ROS_INFO("[PLANNER STATUS : ] Ready for new path \n");
  //     exploration_path_.poses.clear();
  //     path_iterator_ = 0;
  //
  //     graph_status_ = graph_manager_->runLocalPlanner();
  //     switch (graph_status_) {
  //       case GraphManager::GraphStatus::OK:
  //         ROS_INFO("[LOCAL GRAPH STATUS] OK \n");
  //         exploration_path_ = graph_manager_->getLocalExplorationPath();
  //         break;
  //       case GraphManager::GraphStatus::ERR_KDTREE:
  //         ROS_WARN("[LOCAL GRAPH STATUS] Issue occurred with kdtree data.
  //         \n"); break;
  //       case GraphManager::GraphStatus::ERR_NO_FEASIBLE_PATH:
  //         ROS_WARN("[LOCAL GRAPH STATUS] No feasible path was found. \n");
  //         break;
  //       case GraphManager::GraphStatus::NO_GAIN:
  //         ROS_WARN("[LOCAL GRAPH STATUS] Not enough gain \n");
  //         break;
  //       case GraphManager::GraphStatus::NOT_OK:
  //         ROS_WARN("[LOCAL GRAPH STATUS] NOT OK \n");
  //         break;
  //       default:
  //         ROS_WARN(
  //             "[LOCAL GRAPH ERROR] Error occurred in building local graph
  //             \n");
  //         break;
  //     }
  //
  //     if (graph_status_ == GraphManager::GraphStatus::NO_GAIN) {
  //       graph_status_ = graph_manager_->runGlobalPlanner();
  //       switch (graph_status_) {
  //         case GraphManager::GraphStatus::OK:
  //           ROS_INFO("[GLOBAL GRAPH STATUS] OK \n");
  //           exploration_path_ = graph_manager_->getGlobalExplorationPath();
  //           break;
  //         case GraphManager::GraphStatus::NO_GAIN:
  //           ROS_WARN("[GLOBAL GRAPH STATUS] Not enough gain \n");
  //           break;
  //         case GraphManager::GraphStatus::NOT_OK:
  //           ROS_WARN("[GLOBAL GRAPH STATUS] NOT OK \n");
  //           break;
  //         default:
  //           ROS_WARN("[GLOBAL GRAPH STATUS] Error occurred in global
  //           graph."); break;
  //       }
  //     }
  //
  //     if (!exploration_path_.poses.empty()) {
  //       path_iterator_ = 0;
  //       planner_status_ = StageInterface::PlannerStatus::EXECUTING_PATH;
  //     }
  //     break;
  //
  //   case StageInterface::PlannerStatus::EXECUTING_PATH:
  //     ROS_INFO("[PLANNER STATUS : ] Executing path \n");
  //
  //     if (path_iterator_ < (exploration_path_.poses.size())) {
  //       temp_path.poses.clear();
  //       geometry_msgs::PoseStamped p;
  //       if (path_iterator_ == 0) {
  //         p.pose.position.x = robotState_[0];
  //         p.pose.position.y = robotState_[1];
  //         p.pose.position.z = robotState_[2];
  //         p.pose.orientation =
  //             exploration_path_.poses[path_iterator_].pose.orientation;
  //
  //       } else {
  //         p.pose.position.x =
  //             exploration_path_.poses[path_iterator_ - 1].pose.position.x;
  //         p.pose.position.y =
  //             exploration_path_.poses[path_iterator_ - 1].pose.position.y;
  //         p.pose.position.z =
  //             exploration_path_.poses[path_iterator_ - 1].pose.position.z;
  //         p.pose.orientation =
  //             exploration_path_.poses[path_iterator_ - 1].pose.orientation;
  //       }
  //       p.header.stamp = ros::Time::now();
  //       p.header.frame_id = world_frame_id;
  //       geometry_msgs::PoseStamped p1 =
  //       exploration_path_.poses[path_iterator_]; Eigen::Vector3d
  //       v(p.pose.position.x, p.pose.position.y,
  //                         p.pose.position.z);
  //       Eigen::Vector3d v1(p1.pose.position.x, p1.pose.position.y,
  //                          p1.pose.position.z);
  //       MapManager::VoxelStatus vs =
  //           map_manager_->getPathStatus(v, v1, robot_box_size_, true);
  //       if (MapManager::VoxelStatus::kOccupied != vs) {
  //         temp_path.poses.push_back(p);
  //         temp_path.poses.push_back(p1);
  //         path_iterator_ = path_iterator_ + 1;
  //         planner_status_ = StageInterface::PlannerStatus::EXECUTING_PATH;
  //         StageInterface::generateTrajectory(temp_path);
  //         if (!command_path.poses.empty()) {
  //           command_Path_Publisher.publish(command_path);
  //           StageInterface::callActionServer();  // only for ground robot
  //         }
  //       } else {
  //         ROS_ERROR("Path segment not traversable \n");
  //         path_iterator_ = 0;
  //         exploration_path_.poses.clear();
  //         if (homing_planner_triggered_) {
  //           planner_status_ = StageInterface::PlannerStatus::HOMING;
  //         } else {
  //           planner_status_ = StageInterface::PlannerStatus::READY;
  //         }
  //       }
  //     } else {
  //       path_iterator_ = 0;
  //       exploration_path_.poses.clear();
  //
  //       if (homing_planner_triggered_) {
  //         planner_status_ = StageInterface::PlannerStatus::HOMING;
  //       } else {
  //         planner_status_ = StageInterface::PlannerStatus::READY;
  //       }
  //     }
  //     break;
  //
  //   case StageInterface::PlannerStatus::HOMING:
  //     ROS_WARN("Auto homing triggered \n");
  //     exploration_path_.poses.clear();
  //     exploration_path_ = graph_manager_->getHomingPath();
  //     if (!exploration_path_.poses.empty()) {
  //       path_iterator_ = 0;
  //       planner_status_ =
  //       StageInterface::PlannerStatus::EXECUTING_HOMING_PATH;
  //     }
  //     break;
  //
  //   case StageInterface::PlannerStatus::EXECUTING_HOMING_PATH:
  //     ROS_INFO("[PLANNER STATUS : ] Executing homing path \n");
  //
  //     if (path_iterator_ < (exploration_path_.poses.size())) {
  //       temp_path.poses.clear();
  //       geometry_msgs::PoseStamped p;
  //       p.pose.position.x = robotState_[0];
  //       p.pose.position.y = robotState_[1];
  //       p.pose.position.z = robotState_[2];
  //       p.pose.orientation =
  //           exploration_path_.poses[path_iterator_].pose.orientation;
  //       p.header.stamp = ros::Time::now();
  //       p.header.frame_id = world_frame_id;
  //       temp_path.poses.push_back(p);
  //       temp_path.poses.push_back(exploration_path_.poses[path_iterator_]);
  //       path_iterator_ = path_iterator_ + 1;
  //       planner_status_ =
  //       StageInterface::PlannerStatus::EXECUTING_HOMING_PATH;
  //       StageInterface::generateTrajectory(temp_path);
  //       if (!command_path.poses.empty()) {
  //         command_Path_Publisher.publish(command_path);
  //         StageInterface::callActionServer();  // only for ground robot
  //       }
  //     } else {
  //       path_iterator_ = 0;
  //       exploration_path_.poses.clear();
  //       if (homing_planner_triggered_) {
  //         planner_status_ = StageInterface::PlannerStatus::NOT_READY;
  //       }
  //     }
  //     break;
  //
  //   case StageInterface::PlannerStatus::NOT_READY:
  //     ROS_WARN("[PLANNER STATUS] NOT READY \n");
  //     break;
  //
  //   default:
  //     break;
  // }
  //
  // high_resolution_clock::time_point stop_total =
  // high_resolution_clock::now();
  //
  // auto duration_total =
  //     duration_cast<std::chrono::milliseconds>(stop_total - start_total);
  // if (graph_status_ == GraphManager::GraphStatus::OK &&
  //     duration_total.count() > 10) {
  //   ++planning_iteration_;
  //   ROS_INFO_STREAM("\033[;34m[Planning iter -> "
  //                   << planning_iteration_ << "] \t Exec time :"
  //                   << duration_total.count() << " ms \033[0m\n");
  // }
}
