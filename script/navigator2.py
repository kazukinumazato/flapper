#!/usr/bin/env python3


# in this ptogram, we consider peronal space in 3d space is shaped cilinder　with infinite height
# actually ,the radius of the cilinder is determined by the height of the drone relative to the human,
#  but ignore the effect for nowof hand for now


import rospy
import math
from geometry_msgs.msg import Vector3, PoseStamped, Pose, Quaternion
from std_msgs.msg import Empty, Float64
from util import dist
from variables import *
import numpy as np
from scipy.spatial.transform import Rotation as R



class ApproachPhase:
    def __init__(self, deceleration_radius: float = 1.0, dist_close: float = 0.3):
        self.deceleration_radius = deceleration_radius
        self.dist_close = dist_close
        self.phase: str = 'leading'  # 'leading' or 'circling' or 'ascending'

    def approach(self):

        if self.phase == 'leading':
            self.leading_phase()
        elif self.phase == 'circling':
            self.circling_phase()
        return

    # going to the front of the human
    # goal of xy plane distance: between hand and deceleration radius
    # goal of hight : 50% of the eye's height
    # social distance in xy plane is set diferently in different height like (lower tha eye: xxxm, around eye height: xxxm, above eye: xxxm)
    def leading_phase(self) :
        
        return

    # going to above the hand
    # goal of xy plane distance: keep current distance/ but between hand and deceleration radius (min and max)
    # goal of hight : 30cm above the hand/ but gradually increase (for first linearly, but there's space to improve)
    def circling_phase(self) :
        return


class Navigator():
    def __init__(self, deceleration_radius=1.0, dist_close=0.3):
        # [Constructor]
        # deceleration_radius: drone approach to human outside of this radius
        # dist_close: drone stops approaching when the distance between hand and chest is less than this value

        # mocap subscribers
        self.drone_pose_sub = rospy.Subscriber('mocap_node/mocap/flapper/pose',
                                               PoseStamped,
                                               self.drone_pose_sub_callback)
        self.chest_pose_sub = rospy.Subscriber('mocap_node/mocap/chest/pose',
                                               PoseStamped,
                                               self.chest_pose_sub_callback)
        self.hand_pose_sub = rospy.Subscriber('mocap_node/mocap/hand/pose',
                                              PoseStamped,
                                              self.hand_pose_sub_callback)
        # radius to start deceleration
        self.deceleration_radius = deceleration_radius
        # approach control subscribers and publishers
        self.approach_start_sub = rospy.Subscriber('approach_start',
                                                   Empty,
                                                   self.approach_start_sub_callback)
        self.approach_stop_sub = rospy.Subscriber('approach_stop',
                                                  Empty,
                                                  self.approach_stop_sub_callback)
        self.approach_pub = rospy.Publisher('approach', Pose)
        self.distance_chest_drone_on_XY_plane_pub = rospy.Publisher(
            'distance/chest_drone_on_XY_plane', Float64)
        self.distance_chest_hand_on_XY_plane_pub = rospy.Publisher(
            'distance/chest_hand_on_XY_plane', Float64)
        self.distance_hand_drone_on_XY_plane_pub = rospy.Publisher(
            'distance/hand_drone_on_XY_plane', Float64)
        self.circular_pub = rospy.Publisher('circular', Float64)

        self.running = False
        self.dist_close = dist_close

    # Callback functions for subscribers
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

    # Main function to run the approach behavior
    def run(self, timeout=120):
        rospy.loginfo('approach started')
        self.running = True
        start_t = rospy.get_time()
        while rospy.get_time() <= start_t + timeout and self.running:
            chest_pos = self.chest_pose.position
            hand_pos = self.hand_pose.position
            drone_pos = self.drone_pose.position

            distance_chest_drone_on_XY_plane = dist(chest_pos, drone_pos, True)
            distance_chest_hand_on_XY_plane = dist(chest_pos, hand_pos, True)
            distance_hand_drone_on_XY_plane = dist(hand_pos, drone_pos, True)
            # Publish distances for monitoring
            self.distance_chest_drone_on_XY_plane_pub.publish(
                Float64(distance_chest_drone_on_XY_plane))
            self.distance_chest_hand_on_XY_plane_pub.publish(
                Float64(distance_chest_hand_on_XY_plane))
            self.distance_hand_drone_on_XY_plane_pub.publish(
                Float64(distance_hand_drone_on_XY_plane))

            height_above_hand = 0.3  # desired height above the hand

            #! scaling factor for approach speed
            # when drone is close to the hand, slow down more
            scale = 0.3 if distance_hand_drone_on_XY_plane > self.dist_close else 0.3

            goal_dist_z = (hand_pos.z + height_above_hand -
                           drone_pos.z) * scale
            goal_pos_z = hand_pos.z + height_above_hand - goal_dist_z
            goal_ori = self.look_at_quaternion(
                drone_pos, chest_pos)  # make drone face chest

            rospy.loginfo('hand_pos: {}'.format(hand_pos))
            rospy.loginfo('chest_pos: {}'.format(chest_pos))
            rospy.loginfo('drone_pos: {}'.format(drone_pos))

            if distance_chest_drone_on_XY_plane < distance_chest_hand_on_XY_plane:  # r < r_p
                self.circular_pub.publish(1)
                rospy.loginfo('drone is running on circle.')
                vector_c_h = np.array(
                    [hand_pos.x - chest_pos.x, hand_pos.y - chest_pos.y])  # chest to hand
                vector_c_d = np.array(
                    [drone_pos.x - chest_pos.x, drone_pos.y - chest_pos.y])  # chest to drone
                cross = np.cross(vector_c_h, vector_c_d)
                dot = np.dot(vector_c_h, vector_c_d)
                # compute goal angle for circular path
                goal_theta = np.arctan2(cross, dot) * scale
                # rotation matrix
                r_matrix = np.array([[np.cos(goal_theta), -np.sin(goal_theta)],
                                     [np.sin(goal_theta), np.cos(goal_theta)]])
                vector_c_g = r_matrix @ vector_c_h
                # compute goal position on circular path
                goal_pos = Vector3(chest_pos.x + vector_c_g[0],
                                   chest_pos.y + vector_c_g[1],
                                   goal_pos_z)
                self.approach_pub.publish(Pose(goal_pos, goal_ori))
                rospy.loginfo('goal_pos: {}'.format(goal_pos))
            else:
                self.circular_pub.publish(0)
                if distance_chest_drone_on_XY_plane < self.deceleration_radius:  # r_p <= r < r_v
                    # inside deceleration radius
                    goal_dist_x = (hand_pos.x - drone_pos.x) * scale
                    goal_dist_y = (hand_pos.y - drone_pos.y) * scale
                    # move towards hand position with scaling
                    goal_pos = Vector3(hand_pos.x - goal_dist_x,
                                       hand_pos.y - goal_dist_y,
                                       goal_pos_z)
                    self.approach_pub.publish(Pose(goal_pos, goal_ori))
                    rospy.loginfo('goal_pos: {}'.format(goal_pos))
                    rospy.loginfo('goal_dist_y: {}'.format(goal_dist_y))
                else:  # r >= r_v
                    rospy.loginfo('outside of deceleration_radius')
                    vector_d_h = np.array(
                        [hand_pos.x - drone_pos.x, hand_pos.y - drone_pos.y])  # drone to hand
                    vector_d_h_normalized = vector_d_h / \
                        np.linalg.norm(vector_d_h)  # normalize
                    goal_pos = Vector3(drone_pos.x + vector_d_h_normalized[0] * self.deceleration_radius * (1 - scale),
                                       drone_pos.y +
                                       vector_d_h_normalized[1] *
                                       self.deceleration_radius * (1 - scale),
                                       goal_pos_z)
                    self.approach_pub.publish(Pose(goal_pos, goal_ori))
                    rospy.loginfo('vector_d_h: {}'.format(vector_d_h))
                    rospy.loginfo('vector_y: {}'.format(
                        vector_d_h_normalized[1]))
                    rospy.loginfo('goal_pos: {}'.format(goal_pos))
            rospy.sleep(0.1)
        self.running = False
        rospy.loginfo('drone stay')

    def look_at_quaternion(self, r, p, theta_scale=0.1):
        # r: current position (drone)
        # p: target position (chest)
        # theta_scale: scaling factor for rotation angle
        vector_r_p = np.array([p.x - r.x, p.y - r.y])
        e_x = np.array([1, 0])
        cross = np.cross(e_x, vector_r_p)
        dot = np.dot(e_x, vector_r_p)
        chest_theta = np.arctan2(cross, dot)
        drone_ori = self.drone_pose.orientation
        current_theta = R.from_quat(
            [drone_ori.x, drone_ori.y, drone_ori.z, drone_ori.w]).as_euler('xyz')[2]
        goal_theta = (chest_theta - current_theta) * \
            theta_scale + current_theta
        quaternion = R.from_euler('xyz', [0, 0, goal_theta]).as_quat()
        quaternion = Quaternion(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        return quaternion


if __name__ == '__main__':
    rospy.init_node('navigator')
    Navigator()
    rospy.spin()
