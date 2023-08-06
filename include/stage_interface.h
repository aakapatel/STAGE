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
    virtual void odomCallback(const nav_msgs::Odometry::ConstPtr &pos);
    bool fetchExplorationPath ();
    void segmentPath(const std::vector<geometry_msgs::Pose>& path,
                     std::vector<geometry_msgs::Pose>& path_seg);
    void executeCurrentPath ();

  private:
    ros::ServiceClient m_exploration_path_client;
    nav_msgs::Odometry m_currentPose;
    nav_msgs::Path exploration_path_;
    bool planning_status_local = false;
    bool planning_status_global = false;
    int n_seq_;
    double dt_ = 0.5;
   
  };

#endif
