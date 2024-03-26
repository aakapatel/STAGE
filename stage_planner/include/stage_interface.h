#ifndef STAGE_INTERFACE_H
#define STAGE_INTERFACE_H

#include "graph_management.h"

#include <actionlib/client/simple_action_client.h>
#include "stage_planner/pathFollowerActionAction.h"

class StageInterface
{
public:
  StageInterface(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool initializePlanner (stage_planner::initializePlanning::Request &req,
                          stage_planner::initializePlanning::Response &res);

  bool stopPlanner (stage_planner::stopPlanning::Request &req,
                    stage_planner::stopPlanning::Response &res);

  bool homingPlanner (stage_planner::homePlanning::Request &req,
                      stage_planner::homePlanning::Response &res);

  Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond q);
  Eigen::Quaternion<float> Euler2Quaternion(Eigen::Vector3d v);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &pos);
  Eigen::Vector3d fetchCurrentPosition ();
  double calculateDistance (Eigen::Vector3d p1, Eigen::Vector3d p2);
  bool fetchExplorationPath ();
  void segmentPath(const nav_msgs::Path& path,
                   nav_msgs::Path& path_seg);
  void generateTrajectory ();
  

  bool actionDoneCallback(
      const actionlib::SimpleClientGoalState& state,
      const stage_planner::pathFollowerActionResultConstPtr& result);
  void actionActiveCallback();
  void actionFeedbackCallback(
      const stage_planner::pathFollowerActionFeedbackConstPtr& feedback);

  bool reachedEndOfPath ();

  void mainLoop(const ros::TimerEvent& event);

  bool robot_aerial_ = false;

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer run_loop_timer_;

  std::string robot_type_ = "kAerial";
  std::string m_worldFrameId;

  State robotState_;

  Eigen::Quaterniond quat_;

  // Subscribers 
  ros::Subscriber new_poseSubscriber;
  ros::Subscriber m_poseSubscriber;

  // Services
  ros::ServiceClient m_exploration_path_client;
  ros::ServiceServer m_planner_initialization_srv;
  ros::ServiceServer m_planner_stop_srv;
  ros::ServiceServer m_planner_homing_srv;


  actionlib::SimpleActionClient<stage_planner::pathFollowerActionAction> ac_;
  const double serverWatingTimeout = 1.0;


  stage_planner::pathFollowerActionGoal goal;
  std::vector<geometry_msgs::PoseStamped> goal_path;

  std::string m_odometry;
  std::string m_reference_topic;
  ros::Timer m_runTimer;

  ros::Publisher m_trajectory_Publisher;
  ros::Publisher m_command_Path_Publisher;
  ros::Publisher m_reference_Publisher;
  nav_msgs::Odometry m_currentPose;
  nav_msgs::Odometry m_prevPose;
  nav_msgs::Path command_path;
  Eigen::Vector3d m_currentPosition;
  double m_currentHeading;
  Eigen::Vector3d home_position;
  nav_msgs::Path exploration_path_;
  bool planning_status_local = false;
  bool planning_status_global = false;
  bool planning_status_homing = false;
  int n_seq_;
  double dt_ = 0.1;
  bool firstTime;
  bool initialization = false;
  bool explore = false; // For debug only! normally should be false.
  bool stop_planning = false;
  bool return_home = false;
  bool start_planner_first_time = true;

  bool fetch_new_path_ = true;

  double init_x_, init_y_, init_z_; 

  double distance_to_new_path_;
  //reference tracking utilities

};

#endif
