#pragma once
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>

class RobotManager {

public:
  RobotManager(const ros::NodeHandle& nh);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void periodicTimerCallback(const ros::TimerEvent& event);
  

private:
  ros::NodeHandle nh_;
  float current_x_position = 0;
  float current_y_position = 0;
  std::string odometry_topic = "/hummingbird/ground_truth/odometry";
  ros::Subscriber odom_subscriber;
  ros::Timer periodic_timer_;

};
