#include "control_interface.h"
#include <ros/ros.h>

ControlInterface::ControlInterface(ros::NodeHandle nh, ros::NodeHandle nh_private) :
    nh_(nh),
    nh_private_(nh_private)
{
    if(!nh_private_.getParam("state_topic", state_topic))
        state_topic = "state";
    if(!nh_private_.getParam("path_topic", path_topic))
        path_topic = "/command_path";
    if(!nh_private_.getParam("odom_topic", odom_topic))
        odom_topic = "/hummingbird/ground_truth/odometry";
    if(!nh_private_.getParam("wp_topic", wp_topic))
        wp_topic = "/frontier/pose";
    if(!nh_private_.getParam("WP_SIZE", WP_SIZE))
        WP_SIZE = 0.5;
    if(!nh_private_.getParam("ROTATION_TOLERANCE", ROT_ERROR))
        ROT_ERROR = 0.25;
    if(!nh_private_.getParam("reference_topic", pub_topic))
        pub_topic = "/hummingbird/command/pose";
    if(!nh_private_.getParam("frame_id", frame_id))
        frame_id = "world";
    if(!nh_private_.getParam("use_3D", is3D))
        is3D = false;

    //subscribers
    state_sub = nh_.subscribe<std_msgs::String>(state_topic, 1, &ControlInterface::stateCallback, this);
    path_sub = nh_.subscribe<nav_msgs::Path>(path_topic, 1, &ControlInterface::pathCallback, this);
    wp_sub = nh_.subscribe<geometry_msgs::PoseStamped>(wp_topic, 1, &ControlInterface::wpCallback, this);
    odom_sub = nh_.subscribe<nav_msgs::Odometry>(odom_topic, 1, &ControlInterface::odomCallback, this);

    pub_timer = nh_.createTimer(ros::Duration(0.1), &ControlInterface::publishCallback, this);

    ref_pub = nh_.advertise<geometry_msgs::PoseStamped>(pub_topic, 1);
    point_reatch_pub = nh_.advertise<std_msgs::Bool>("yaw_error" , 1);

    for(int i = 0; i < 200; i++){
        ref_path.data.push_back(0.0);
    }
}

void ControlInterface::stateCallback(const std_msgs::String::ConstPtr& msg){

    if(msg->data == "HOLD"){
        state = HOLD;
        hold_now = true;
    }
    else if(msg->data == "PATH") state = PATH;
    else if(msg->data == "POINT") state = POINT;
    else if(msg->data == "ROTATE") state = ROTATE;
    else ROS_WARN("Invalid state: %s\nWalid are:\nHOLD\nPATH\nPOINT\nROTATE", msg->data.c_str());
}

void ControlInterface::publishCallback(const ros::TimerEvent&){

    switch(state){
        case HOLD:
            if(hold_now){
                ref_pub.publish(odom);
                hold_now = false;
            }
            break;

        case PATH:
            if(path_exist){
                ControlInterface::updatePath();
                path_pose.header.frame_id = frame_id;
                ref_pub.publish(path_pose);
                ROS_INFO_STREAM ("Publishing Reference " << "\n");
            }
            break;

        case POINT:
            if(point_set){
                ref_pub.publish(point);
                if(ControlInterface::atPoint(point.pose.position)){
                    std_msgs::Bool rot_msg;
                    rot_msg.data = true;
                    point_reatch_pub.publish(rot_msg);
                    //point_set = false;
                }
            }
            break;

        case ROTATE:
            if(point_set){
                geometry_msgs::PoseStamped rot_point = odom;
                rot_point.pose.orientation = ControlInterface::quartToPoint(point.pose.position);
                ref_pub.publish(rot_point);
                if(ControlInterface::atRotation(rot_point.pose.orientation)){
                    point_set = false;
                }
            }
            break;

        default:
            ROS_ERROR("invalide state: %i", state);
            break;
    }
}

void ControlInterface::pathCallback(const nav_msgs::PathConstPtr& msg){
    path = *msg;
    index = 0;
    path_exist = true;
    ROS_INFO ("Path Received");
    ControlInterface::setPose();
    return;
}

void ControlInterface::wpCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    point = *msg;
    point_set = true;
    return;
}

void ControlInterface::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    odom.header = msg->header;
    odom.pose = msg->pose.pose;

}

void ControlInterface::updatePath(){
    while( ControlInterface::atPoint(path_pose.pose.position)){
        if (path.poses.size() > index + 1){
            index++;
            ControlInterface::setPose();
        }
        else {
            path_exist = false; 
            return;
        }
    }
}

void ControlInterface::setPose(){
    path_pose = path.poses[index];
    // path_pose.pose.orientation = ControlInterface::quartToPoint(path_pose.pose.position);

}

geometry_msgs::Quaternion ControlInterface::quartToPoint(geometry_msgs::Point point){
    double x = -odom.pose.position.x + point.x;
    double y = -odom.pose.position.y + point.y;
    double z = -odom.pose.position.z + point.z;

    double pith = asin(z);
    double yaw = atan2(y, x);
    double rol = 0.0;

    double c1 = cos(yaw / 2);
    double c2 = cos(pith / 2);
    double c3 = cos(rol / 2);
    double s1 = sin(yaw / 2);
    double s2 = sin(pith / 2);
    double s3 = sin(rol / 2);

    geometry_msgs::Quaternion quat;

    quat.w = 1;
    quat.x = 0;
    quat.y = 0;
    quat.z = yaw;
    return quat;

}

bool ControlInterface::atPoint(geometry_msgs::Point point){
    if (is3D){
        return sqrt(pow(odom.pose.position.x - point.x, 2) + 
        pow(odom.pose.position.y - point.y, 2) + pow(odom.pose.position.z - point.z, 2) )  < WP_SIZE;
    } else {
    return sqrt( pow(odom.pose.position.x - point.x, 2) + 
        pow(odom.pose.position.y - point.y, 2)  + pow(odom.pose.position.z - point.z, 2) ) < WP_SIZE;
    }
    return false;
}

bool ControlInterface::atRotation(geometry_msgs::Quaternion q){
    double yaw_current = atan2(2.0*(odom.pose.orientation.w * odom.pose.orientation.z + odom.pose.orientation.x * odom.pose.orientation.y) , (1.0 - 2.0 * (odom.pose.orientation.y * odom.pose.orientation.y + odom.pose.orientation.z * odom.pose.orientation.z)));

    double yaw_waypoint = atan2(2.0*(q.w * q.z + q.x * q.y) , (1.0 - 2.0 * (q.y * q.y + q.z * q.z)));

    return abs(yaw_current - yaw_waypoint) < ROT_ERROR;
}


int main(int argc, char **argv){
    ros::init (argc, argv, "control_interface");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ControlInterface controlInterface (nh, nh_private);

    ros::spin();

    return 0;
}
