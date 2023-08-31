#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <math.h>


enum STATE {
    HOLD,
    PATH,
    POINT,
    ROTATE
};


class ControlInterface
{
public:
    ControlInterface(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber state_sub;
    ros::Subscriber path_sub;
    ros::Subscriber wp_sub;
    ros::Subscriber odom_sub;
    ros::Timer pub_timer;

    ros::Publisher ref_pub;
    ros::Publisher point_reatch_pub;

    std::string state_topic;
    std::string pub_topic;
    std::string path_topic;
    std::string wp_topic;
    std::string odom_topic;

    std::string frame_id;
    bool is3D;

    geometry_msgs::PoseStamped odom;
    nav_msgs::Path path;
    geometry_msgs::PoseStamped path_pose;
    geometry_msgs::PoseStamped point;
    std_msgs::Float32MultiArray ref_path;

    bool path_exist = false;
    bool point_set = false;
    bool hold_now = false;
    bool first_path;
    

    double WP_SIZE; // squar radus of wp/point in path
    double ROT_ERROR;
    int index = 0;
    int path_len;

    void stateCallback(const std_msgs::String::ConstPtr& msg);
    void pathCallback(const nav_msgs::PathConstPtr& msg);
    void wpCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    void publishCallback(const ros::TimerEvent&);

    void setPose();
    void updatePath();
    geometry_msgs::Quaternion quartToPoint(geometry_msgs::Point point);
    bool atPoint(geometry_msgs::Point point);
    bool atRotation(geometry_msgs::Quaternion q);

    // TODO NOT SAFE!!!!!! CHANGE THIS TO HOLD 

    STATE state = HOLD; //DEBUG ONLY. IT SHOULD BE HOLD MODE

};
