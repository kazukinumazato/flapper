#!/usr/bin/env pytthon

import rospy
import math
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped
from variables import *

class Navigator():
    def __init__(self, drone_client, safety_radius = 0.4):
        self.state_pub = rospy.Publisher('state', Int64, queue_size=1)
        self.drone_pose_sub = rospy.Subscriber('mocap_node/mocap/flapper/pose',
                                               PoseStamped,
                                               self.drone_pose_sub_callback)              
        self.chest_pose_sub = rospy.Subscriber('mocap_node/mocap/chest/pose',
                                               PoseStamped,
                                               self.chest_pose_sub_callback)
        self.hand_pose_sub = rospy.Subscriber('mocap_node/mocap/hand/pose',
                                              PoseStamped,
                                              self.hand_pose_sub_callback)
        self.drone_client = drone_client
        self.safety_radius = safety_radius

    def drone_pose_sub_callback(self, msg):
        self.drone_pose = msg.pose
        self.drone_client.send_pose(self.drone_pose.position, self.drone_pose.orientation)
        
    def chest_pose_sub_callback(self, msg):
        self.chest_pose = msg.pose
        
    def hand_pose_sub_callback(self, msg):
        self.hand_pose = msg.pose

    def takeoff(self, height = 1, duration = 3, threshold = 0.01, timeout = 10):
        if not hasattr(self, 'drone_pose'):
            rospy.logwarn('drone pose is not acquired yet. Unable to take off.')
            return
        rospy.loginfo('takeoff started')
        self.drone_client.takeoff(height, duration)
        self.state_pub.publish(RobotState.TAKEOFF)
        start_t = rospy.get_time()
        while rospy.get_time() <= start_t + timeout:
            rospy.sleep(0.1)
            if abs(self.drone_pose.position.z - height) <= threshold:
                rospy.sleep(2)
                rospy.loginfo('takeoff succeeded')
                self.state_pub.publish(RobotState.HOVER)
                break
            if rospy.is_shutdown():
                print('rospy shutdown')
                break

    def land(self, duration = 3):
        rospy.loginfo('land started')
        self.drone_client.land(0, duration)
        self.state_pub.publish(RobotState.LAND)
        rospy.sleep(3)
        rospy.loginfo('land succeeded')
        self.state_pub.publish(RobotState.STOP)

    def stop(self):
        self.drone_client.stop()
        rospy.loginfo('drone stopped')
        self.state_pub.publish(RobotState.STOP)

    def approach(self, timeout = 180):
        def dist(pos1, pos2, plane = False):
            if plane:
                return math.sqrt((pos1.x - pos2.x)**2 +
                                 (pos1.y - pos2.y)**2)
            return math.sqrt((pos1.x - pos2.x)**2 +
                             (pos1.y - pos2.y)**2 +
                             (pos1.z - pos2.z)**2)
        rospy.loginfo('approach started')
        start_t = rospy.get_time()
        self.state_pub.publish(RobotState.APPROACH)
        while rospy.get_time() <= start_t + timeout:
            chest_pos = self.chest_pose.position
            hand_pos = self.hand_pose.position
            drone_pos = self.drone_pose.position
            distance_chest_hand = dist(chest_pos, hand_pos)
            distance_hand_drone_on_XY_plane = dist(hand_pos, drone_pos, True)
            rospy.loginfo(f'distance_chest_hand: {distance_chest_hand} distance_hand_drone:{distance_hand_drone_on_XY_plane}')
            if distance_hand_drone_on_XY_plane < 0.05:
                rospy.loginfo('palm land is ready')
                self.state_pub.publish(RobotState.PALM_LAND_READY)
                break
            elif distance_chest_hand > 0.5:
                deltaX = (hand_pos.x - drone_pos.x) / 10
                deltaY = (hand_pos.y - drone_pos.y) / 10
                deltaZ = (hand_pos.z + 0.5 - drone_pos.z) / 10
                self.drone_client.go_to(deltaX,
                                        deltaY,
                                        deltaZ, 0, 0.1, True)
                # self.drone_client.go_to(hand_pos.x, hand_pos.y, hand_pos.z, 0, 5)
            rospy.sleep(0.5)

    def palm_land(self, duration = 3):
        rospy.loginfo('palm land started')
        self.state_pub.publish(RobotState.PALM_LAND)
        hand_height = self.hand_pose.position.z
        self.drone_client.land(hand_height, duration)
        rospy.sleep(duration)
        rospy.loginfo('palm land succeeded')
        self.state_pub.publish(RobotState.STOP)
