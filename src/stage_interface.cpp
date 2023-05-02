#include "stage_interface.h"

using namespace std;

StageInterface::StageInterface(ros::NodeHandle private_nh_)
    : m_nh(),
    m_odometry("/hummingbird/ground_truth/odometry")
    // cloud_in("/hummingbird/velodyne_points"),
   {

    ros::NodeHandle private_nh = private_nh_;
    //Set variables
    private_nh.param("odometry", m_odometry, m_odometry);


    //Register to the subscriber topic and register it to the callback function

    m_poseSubscriber = new message_filters::Subscriber<nav_msgs::Odometry> (m_nh, m_odometry, 5);
    m_poseSubscriber->registerCallback(boost::bind(&StageInterface::odomCallback, this, _1));

    //Register publishers
};


void StageInterface::odomCallback(const nav_msgs::Odometry::ConstPtr& pose) {

    m_currentPose = *pose;
    // std::cout << "current position X " << m_currentPose.pose.pose.position.x << std::endl;
}

