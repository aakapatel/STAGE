#ifndef STAGE_INTERFACE_H
#define STAGE_INTERFACE_H

#include <graph_management.h>

class StageInterface {
public:
    StageInterface(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
    ros::NodeHandle m_nh;
    message_filters::Subscriber<nav_msgs::Odometry>* m_poseSubscriber;
    std::string m_odometry;
    virtual void odomCallback(const nav_msgs::Odometry::ConstPtr& pos);
    nav_msgs::Odometry m_currentPose;

};




#endif
