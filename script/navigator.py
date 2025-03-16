#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import Empty
from util import dist
from variables import *
import numpy as np

class Navigator():
    def __init__(self, safety_radius = 0.4):
        self.drone_pose_sub = rospy.Subscriber('mocap_node/mocap/flapper/pose',                 
                                               PoseStamped,
                                               self.drone_pose_sub_callback)
        self.chest_pose_sub = rospy.Subscriber('mocap_node/mocap/chest/pose',                      
                                               PoseStamped,                                         
                                               self.chest_pose_sub_callback)
        self.hand_pose_sub = rospy.Subscriber('mocap_node/mocap/hand/pose',
                                              PoseStamped,                                          
                                              self.hand_pose_sub_callback)                          
        self.safety_radius = safety_radius
        self.approach_start_sub = rospy.Subscriber('approach_start',
                                                   Empty,
                                                   self.approach_start_sub_callback)
        self.approach_stop_sub = rospy.Subscriber('approach_stop',
                                                  Empty,
                                                  self.approach_stop_sub_callback)
        self.approach_pub = rospy.Publisher('approach', Vector3)
        self.approach_dummy_pub = rospy.Publisher('approach_dummy', Vector3)
        self.running = False

    def drone_pose_sub_callback(self, msg):
        self.drone_pose = msg.pose

    def chest_pose_sub_callback(self, msg):
        self.chest_pose = msg.pose

    def hand_pose_sub_callback(self, msg):
        self.hand_pose = msg.pose

    def approach_start_sub_callback(self, msg):
        if not self.running:
            self.run()

    def approach_stop_sub_callback(self, msg):
        if self.running:
            self.running = False
        
    def run(self, timeout=120):
        rospy.loginfo('approach started')
        self.running = True
        start_t = rospy.get_time()
        while rospy.get_time() <= start_t + timeout and self.running:
            scale = 0.8
            chest_pos = self.chest_pose.position
            hand_pos = self.hand_pose.position
            drone_pos = self.drone_pose.position
            distance_chest_drone_on_XY_plane = dist(chest_pos, drone_pos, True)
            distance_chest_hand_on_XY_plane = dist(chest_pos, hand_pos, True)
            height_above_hand = 0.4
            goal_dist_z = (hand_pos.z + height_above_hand - drone_pos.z) * scale
            goal_pos_z = hand_pos.z + height_above_hand - goal_dist_z
            if distance_chest_drone_on_XY_plane < distance_chest_hand_on_XY_plane:
                rospy.loginfo('drone is running on circle.')
                vector_c_h = np.array([hand_pos.x - chest_pos.x, hand_pos.y - chest_pos.y])
                vector_c_d = np.array([drone_pos.x - chest_pos.x, drone_pos.y - chest_pos.y])
                cross = np.cross(vector_c_h, vector_c_d)
                dot = np.dot(vector_c_h, vector_c_d)
                goal_theta = np.arctan2(cross, dot) * scale
                r_matrix = np.array([[np.cos(goal_theta), -np.sin(goal_theta)],
                                     [np.sin(goal_theta), np.cos(goal_theta)]])
                vector_c_g = r_matrix @ vector_c_h
                goal_pos = Vector3(chest_pos.x + vector_c_g[0],
                                   chest_pos.y + vector_c_g[1],
                                   goal_pos_z)
                self.approach_pub.publish(goal_pos)
                rospy.loginfo('goal_pos: {}'.format(goal_pos))
            else:
                goal_dist_x = (hand_pos.x - drone_pos.x) * scale
                goal_dist_y = (hand_pos.y - drone_pos.y) * scale
                goal_pos = Vector3(hand_pos.x - goal_dist_x,
                                   hand_pos.y - goal_dist_y,
                                   goal_pos_z)
                self.approach_pub.publish(goal_pos)
                rospy.loginfo('goal_pos: {}'.format(goal_pos))
            rospy.sleep(0.1)
        self.running = False
        rospy.loginfo('drone stay')

if __name__ == '__main__':
    rospy.init_node('navigator')
    Navigator()
    rospy.spin()
    
