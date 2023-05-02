#!/usr/bin/env python

import rospy, time
from rospy import timer
from std_msgs.msg import String
import std_msgs.msg
from nav_msgs.msg import Path
from math import pi, sin, cos, atan2, sqrt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, Pose, PointStamped, Point
from geometry_msgs.msg import PoseStamped
from acl_exploration.msg import path
import time
import subprocess

import math

WP_SIZE = 0.6

return_flag = 0
m = 0


class Server:



    def __init__(self, current_time=None):

        global mission_timer, mission_duration, start
        mission_timer = time.time()

        self.waypoint = False
        self.frontier_received = False
        self.xbee_return_flag = False
        self.request_homing = False
        self.land_flag = False
        self.local_heading_ref = 0
        self.heading_ref = 0
        self.x_c = 0
        self.y_c = 0
        self.z_c = 0
        self.x_r = 0
        self.y_r = 0
        self.z_r = 0
        self.d = 0

        #############################################################################################

        mission_duration = 80000

        #############################################################################################

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.vel_publisher = rospy.Publisher('/hummingbird/reference', PoseStamped, queue_size = 1)

        self.vel_msg = PoseStamped()

        self.safety_publisher = rospy.Publisher('/hummingbird/safety_land', String, queue_size= 10)

        self.safety_msg = String()

        self.return_publisher = rospy.Publisher('/return_home', String, queue_size= 10)
        self.return_command = String()

        self.explore_publisher = rospy.Publisher('/explore', String, queue_size= 10)
        self.explore_command = String()

        rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, self.read_callback)

        rospy.Subscriber('/return_home',  String, self.return_to_base)

        self.waypoint_subscriber = rospy.Subscriber('/hummingbird/exploration_path', path, self.waypoint_callback)

        self.f_x = 0
        self.f_y = 0
        self.f_z = 0

    def frontier_callback(self, msg):

        self.f_x = msg.x
        self.f_y = msg.y
        self.f_z = msg.z
        self.frontier_received = True

    def return_to_base(self, msg):

        self.xbee_return_flag = True

    def waypoint_callback(self, msg):

        self.wp_index = 0

        self.path = msg.path
        self.path_type = msg.type.data
        self.x_r = msg.path.poses[self.wp_index].pose.position.x
        self.y_r = msg.path.poses[self.wp_index].pose.position.y
        self.z_r = msg.path.poses[self.wp_index].pose.position.z
        self.waypoint = True

    def navigate(self, current_time=None):

        while (sqrt((self.x_c - self.x_r) * (self.x_c - self.x_r)  + (self.y_c - self.y_r) * (self.y_c - self.y_r) + (self.z_c - self.z_r) * (self.z_c - self.z_r)) < WP_SIZE):
            if(self.wp_index < len(self.path.poses) - 1):
                self.wp_index = self.wp_index + 1
            else:
                break
            self.x_r = self.path.poses[self.wp_index].pose.position.x
            self.y_r = self.path.poses[self.wp_index].pose.position.y
            self.z_r = self.path.poses[self.wp_index].pose.position.z

        self.vel_msg.header = std_msgs.msg.Header()
        self.vel_msg.header.stamp = rospy.Time.now()
        self.vel_msg.header.frame_id = 'world'
        self.heading_ref = math.atan2((self.y_r - self.y_c), (self.x_r - self.x_c))
        self.vel_msg.pose.position.x = self.x_r
        self.vel_msg.pose.position.y = self.y_r
        self.vel_msg.pose.position.z = self.z_r
        self.vel_msg.pose.orientation.x = 0
        self.vel_msg.pose.orientation.y = 0
        # self.vel_msg.pose.orientation.z = self.path.poses[self.wp_index].pose.orientation.z
        self.vel_msg.pose.orientation.z = self.heading_ref
        self.vel_msg.pose.orientation.w = 0
        self.vel_publisher.publish(self.vel_msg)

        return

    def read_callback(self, msg):

        self.x_c = msg.pose.pose.position.x
        self.y_c = msg.pose.pose.position.y
        self.z_c = msg.pose.pose.position.z

        start = time.time()

        self.d = sqrt((self.x_c)*(self.x_c) + (self.y_c)*(self.y_c) + (self.z_c)*(self.z_c))


        if ((start - mission_timer) < mission_duration):

            print("%.2f" % (start - mission_timer))

            self.explore_publisher.publish(self.explore_command)

            if (self.xbee_return_flag == False):

                if (self.waypoint and self.path_type == "local"):
                    self.navigate()
                    print("Local Exploration ..")

                if (self.waypoint and self.path_type == "global"):
                    self.navigate()
                    print("Reposition to GLobal Vertex ..")


            else:
                self.request_homing = True
                self.return_publisher.publish(self.return_command)
                print("Homing path requested")
                if (self.waypoint and self.path_type == "homing"):
                    self.navigate()
                    print("Returning To Base ----    ^_^ ^_^ ")

                if (self.d < 1.5):
                    self.safety_publisher.publish(self.safety_msg)
                    self.land_flag = True
                    print("Reached Base, safety landing....")


        else:

            self.return_publisher.publish(self.return_command)

            print("Homing path requested")

            if (self.waypoint and self.path_type == "homing"):
                self.navigate()
                print("Returning To Base ----    ^_^ ^_^ ")

                if (self.d < 1.5):
                    self.safety_publisher.publish(self.safety_msg)
                    self.land_flag = True
                    print("Reached Base, safety landing....")


        end = time.time()

        return


if __name__ == "__main__":
    try:
        rospy.init_node('Active Loop Closure Exploration', anonymous=True)
        rospy.loginfo("Starting...")

        server = Server()

        while not rospy.is_shutdown():
            rospy.spin()

        rospy.loginfo("If not at base, I am in trouble... send searh and rescue !!!! ")
    except rospy.ROSInterruptException:
        pass
