#ifndef STAGE_INTERFACE_H
#define STAGE_INTERFACE_H

#include "graph_management.h"

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

    Eigen::Quaternion<float> Euler2Quaternion(ufo::math::Vector3 v);
    void updateCurrentPosition(const nav_msgs::Odometry::ConstPtr &pos);
    ufo::math::Vector3 fetchCurrentPosition ();
    double calculateDistance (ufo::math::Vector3 p1, ufo::math::Vector3 p2);
    bool fetchExplorationPath ();
    void segmentPath(const nav_msgs::Path& path,
                                 nav_msgs::Path& path_seg);
    void generateTrajectory ();
    bool reachedEndOfPath ();
    
    void mainLoop(const ros::TimerEvent& event);


  private:
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
   
    ros::Timer run_loop_timer_;

    // Subscribers 
    ros::Subscriber new_poseSubscriber;
    ros::Subscriber m_poseSubscriber;

    // Services
    ros::ServiceClient m_exploration_path_client;
    ros::ServiceServer m_planner_initialization_srv;
    ros::ServiceServer m_planner_stop_srv;
    ros::ServiceServer m_planner_homing_srv;


    std::string m_odometry;
    std::string m_reference_topic;
    ros::Timer m_runTimer;

    ros::Publisher m_trajectory_Publisher;
    ros::Publisher m_command_Path_Publisher;
    ros::Publisher m_reference_Publisher;
    nav_msgs::Odometry m_currentPose;
    nav_msgs::Odometry m_prevPose;
    nav_msgs::Path command_path;
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
    bool explore = false; // For debug only! normally should be false.
    bool stop_planning = false;
    bool return_home = false;
    bool start_planner_first_time = true;

    bool fetch_new_path_ = true;
    
    //reference tracking utilities

  };

#endif
