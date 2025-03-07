#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Vector3, PoseStamped
from std_msgs.msg import Empty
from util import dist
from variables import *

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
            chest_pos = self.chest_pose.position
            hand_pos = self.hand_pose.position
            drone_pos = self.drone_pose.position
            distance_chest_drone_on_XY_plane = dist(chest_pos, drone_pos, True)
            if distance_chest_drone_on_XY_plane < self.safety_radius:
                rospy.logwarn('chest and drone are too close. drone is going away.')
                deltaX = drone_pos.x - chest_pos.x
                deltaY = drone_pos.y - chest_pos.y
                norm = math.sqrt(deltaX ** 2 + deltaY ** 2)
                deltaX /= (norm * 5)
                deltaY /= (norm * 5)
                goal_pos = Vector3(drone_pos.x + deltaX,
                                   drone_pos.y + deltaY,
                                   drone_pos.z)
                self.approach_pub.publish(goal_pos)
            else:
                deltaX = (hand_pos.x - drone_pos.x) / 10
                deltaY = (hand_pos.y - drone_pos.y) / 10
                deltaZ = (hand_pos.z + 0.2 - drone_pos.z) / 10
                goal_pos = Vector3(drone_pos.x + deltaX,
                                   drone_pos.y + deltaY,
                                   drone_pos.z + deltaZ)
                self.approach_pub.publish(goal_pos)
            rospy.sleep(0.1)
        self.running = False
        rospy.loginfo('drone stay')

if __name__ == '__main__':
    rospy.init_node('navigator')
    Navigator()
    rospy.spin()
    
