#include "test.h"
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>

RobotManager::RobotManager (const ros::NodeHandle& nh) 
  : nh_(nh) 
{

  odom_subscriber = nh_.subscribe("/hummingbird/ground_truth/odometry", 1000, &RobotManager::odom_callback, this);

  periodic_timer_ = nh_.createTimer(ros::Duration(1),
                                       &RobotManager::periodicTimerCallback,
                                       this);
}

void RobotManager::odom_callback (const nav_msgs::Odometry::ConstPtr &msg) {

  current_x_position = msg->pose.pose.position.x;
  current_y_position = msg->pose.pose.position.y;

  // ROS_INFO_STREAM ("Robot position : " << current_x_position << " , " << current_y_position << " \n" );

}

void RobotManager::periodicTimerCallback(const ros::TimerEvent &event) {
  ros::Duration(10).sleep();
  ROS_INFO_STREAM ("Timer prints this information .. ");
}
