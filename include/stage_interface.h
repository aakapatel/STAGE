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
    virtual void updateCurrentPosition(const nav_msgs::Odometry::ConstPtr &pos);
    ufo::math::Vector3 fetchCurrentPosition ();
    double calculateDistance ();
    bool fetchExplorationPath ();
    void segmentPath(const nav_msgs::Path& path,
                                 nav_msgs::Path& path_seg.poses);
    void executeCurrentPath ();

  private:
    ros::ServiceClient m_exploration_path_client;
    nav_msgs::Odometry m_currentPose;
    nav_msgs::Odometry m_prevPose;
    ufo::math::Vector3 m_currentPosition;
    double m_currentHeading;
    ufo::math::Vector3 home_position;
    nav_msgs::Path exploration_path_;
    bool planning_status_local = false;
    bool planning_status_global = false;
    int n_seq_;
    double dt_ = 0.5;
   
  };

#endif
