#ifndef STAGE_INTERFACE_H
#define STAGE_INTERFACE_H

#include "graph_management.h"

class StageInterface
{
  public:
    StageInterface(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
    ros::NodeHandle m_nh;
    message_filters::Subscriber<nav_msgs::Odometry> *m_poseSubscriber;
    std::string m_odometry;

    
    bool initializePlanner (stage_planner::initializePlanning::Request &req,
                                        stage_planner::initializePlanning::Response &res);
    
    bool stopPlanner (stage_planner::stopPlanning::Request &req,
                                        stage_planner::stopPlanning::Response &res);

    bool homingPlanner (stage_planner::homePlanning::Request &req,
                                        stage_planner::homePlanning::Response &res);

    Eigen::Quaternion<float> Euler2Quaternion(ufo::math::Vector3 v);
    void updateCurrentPosition(const nav_msgs::Odometry::ConstPtr &pos);
    ufo::math::Vector3 fetchCurrentPosition ();
    double calculateDistance (ufo::math::Vector3 p1, ufo::math::Vector3 p2);
    void fetchExplorationPath ();
    void segmentPath(const nav_msgs::Path& path,
                                 nav_msgs::Path& path_seg);
    void executeCurrentPath ();
    bool reachedEndOfPath ();

  private:
    ros::ServiceClient m_exploration_path_client;
    ros::ServiceServer m_planner_initialization_srv;
    ros::ServiceServer m_planner_stop_srv;
    ros::ServiceServer m_planner_homing_srv;

    ros::Publisher m_trajectory_Publisher;
    ros::Publisher m_command_Path_Publisher;
    nav_msgs::Odometry m_currentPose;
    nav_msgs::Odometry m_prevPose;
    geometry_msgs::PoseArray command_path;
    ufo::math::Vector3 m_currentPosition;
    double m_currentHeading;
    ufo::math::Vector3 home_position;
    nav_msgs::Path exploration_path_;
    bool planning_status_local = false;
    bool planning_status_global = false;
    bool planning_status_homing = false;
    int n_seq_;
    double dt_ = 0.5;
    bool firstTime;
    bool initialization = false;
    bool explore = false;
    bool stop_planning = false;
    bool return_home = false;
   
  };

#endif
