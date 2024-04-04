#include <ros/ros.h>
#include "path_tracker.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spot_path_tracker_node");

	ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

	PathTracker ppc(nh, nh_private);

	ros::spin();
}
