#include "test.h"
#include <ros/ros.h>

int main (int argc, char **argv) {

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    RobotManager robotManager = RobotManager(nh);
    ros::spin();

}
