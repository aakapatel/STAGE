#ifndef STAGE_INTERFACE_H
#define STAGE_INTERFACE_H

#include <actionlib/client/simple_action_client.h>

#include "graph_management.h"
#include "stage_planner/pathFollowerActionAction.h"

class StageInterface {
 public:
  enum PlannerStatus {
    READY = 0,
    EXECUTING_PATH,
    EXECUTING_HOMING_PATH,
    NOT_READY,
    HOMING
  };

  StageInterface(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                 MapManagerVoxblox<MapManagerVoxbloxServer,
                                   MapManagerVoxbloxVoxel>* map_manager);

  void initializeAttributes();
  void loadParams();
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params,
                       const BoundedSpaceParams& local_space_params,
                       const BoundedSpaceParams& manual_mode_params);
  double projectSample(Eigen::Vector3d& sample,
                       MapManager::VoxelStatus& voxel_status);
  bool initializePlanner(stage_planner::initializePlanning::Request& req,
                         stage_planner::initializePlanning::Response& res);

  bool stopPlanner(stage_planner::stopPlanning::Request& req,
                   stage_planner::stopPlanning::Response& res);

  bool homingPlanner(stage_planner::homePlanning::Request& req,
                     stage_planner::homePlanning::Response& res);

  Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond q);
  Eigen::Quaternion<float> Euler2Quaternion(Eigen::Vector3d v);
  void odometryCallback(const nav_msgs::Odometry& odom);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);
  double calculateDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);
  void segmentPath(const nav_msgs::Path& path, nav_msgs::Path& path_seg);
  void generateTrajectory(nav_msgs::Path);

  bool actionDoneCallback(
      const actionlib::SimpleClientGoalState& state,
      const stage_planner::pathFollowerActionResultConstPtr& result);
  void actionActiveCallback();
  void actionFeedbackCallback(
      const stage_planner::pathFollowerActionFeedbackConstPtr& feedback);
  bool reachedEndOfPath();
  void callActionServer();

  void periodicTimerCallback(const ros::TimerEvent& event);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
      map_manager_;
  actionlib::SimpleActionClient<stage_planner::pathFollowerActionAction> ac_;
  const double serverWatingTimeout = 1.0;
  stage_planner::pathFollowerActionGoal goal;

  GraphManager* graph_manager_;
  VisualizationServer* visualization_server_;
  std::string sensor_name;
  std::string world_frame_id;
  SensorParams sensor_params_;
  SensorParams free_frustum_params_;
  RobotParams robot_params_;
  BoundedSpaceParams local_space_params_;
  BoundedSpaceParams manual_mode_params_;
  BoundedSpaceParams global_space_params_;
  PlanningParams planning_params_;
  std::shared_ptr<SampleStatistic> stat_, stat_chrono_;

  StateVec robotState_;
  Eigen::Quaterniond quat_;
  Eigen::Vector3d robot_box_size_;
  // status
  StageInterface::PlannerStatus planner_status_ =
      StageInterface::PlannerStatus::NOT_READY;
  GraphManager::GraphStatus graph_status_ = GraphManager::GraphStatus::NOT_OK;
  // Subscribers
  ros::Subscriber odometry_Subscriber_;
  ros::Subscriber goalSubscriber_;

  // Services
  ros::ServiceServer planner_initialization_service_;
  ros::ServiceServer planner_stop_service_;
  ros::ServiceServer planner_homing_service_;
  ros::ServiceClient planner_takeoff_service_;
  ros::ServiceClient planner_land_service_;
  ros::ServiceClient planner_arming_client_;
  ros::ServiceClient planner_set_mode_client_;
  // Timer

  ros::Time rostime_start_;
  ros::Timer planner_periodic_timer_;
  double time_elapsed = 0.0;

  std::vector<geometry_msgs::PoseStamped> goal_path;

  ros::Publisher trajectory_Publisher;
  ros::Publisher command_Path_Publisher;
  ros::Publisher reference_Publisher;
  ros::Publisher controller_Start_Publisher;
  ros::Publisher controller_Land_Publisher;
  nav_msgs::Path command_path;
  nav_msgs::Path exploration_path_;
  nav_msgs::Path temp_path;
  bool planning_status_local = false;
  bool planning_status_global = false;
  bool planning_status_homing = false;

  int n_seq_;
  double v_max_;
  double yaw_rate_max_;
  double dt_ = 0.1;
  double init_x = 2;
  double time_budget_remaining_ = 0;
  int path_iterator_ = 0;
  int planning_iteration_ = 0;

  bool firstTime;
  bool planner_initialized = false;
  bool received_init_location = false;
  bool manual_homing_triggered_ = false;
  bool homing_planner_triggered_ = false;
  bool explore = false;  // For debug only! normally should be false.
  bool stop_planning = false;
  bool return_home = false;
  bool start_planner_first_time = true;
  bool path_execution_complete_ = false;
  bool odom_received = false;
  bool fetch_new_path_ = true;

  double init_x_, init_y_, init_z_;

  // reference tracking utilities
};

#endif
