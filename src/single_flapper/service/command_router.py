#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from script.shared.variables import *
from scipy.spatial.transform import Rotation as R
import numpy as np
from script.shared.util import extract_yaw


"""
トピックをもとにコマンド指令を行うルーター
"""


class CommandRouter:
    def __init__(self, motion_manager):
        self.motion_manager = motion_manager

        self.takeoff_sub = rospy.Subscriber(
            "teleop_command/takeoff", Empty, self.takeoff_sub_callback
        )
        self.land_sub = rospy.Subscriber(
            "teleop_command/land", Empty, self.land_sub_callback
        )
        self.stop_sub = rospy.Subscriber(
            "teleop_command/halt", Empty, self.stop_sub_callback
        )

        self.palm_land_sub = rospy.Subscriber(
            "palm_land", Pose, self.palm_land_sub_callback
        )
        self.approach_sub = rospy.Subscriber(
            "approach", Pose, self.approach_sub_callback
        )

    def takeoff_sub_callback(self, msg):
        self.motion_manager.takeoff()

    def land_sub_callback(self, msg):
        self.motion_manager.land()

    def stop_sub_callback(self, msg):
        self.motion_manager.stop()

    def palm_land_sub_callback(self, msg):
        yaw = extract_yaw(msg.orientation)
        rospy.loginfo("Palm land with yaw: %f" % np.rad2deg(yaw))
        self.motion_manager.palm_land(yaw)

    def approach_sub_callback(self, msg):
        yaw = extract_yaw(msg.orientation)
        rospy.loginfo(
            "Approach to x: %f, y: %f, z: %f, yaw: %f"
            % (msg.position.x, msg.position.y, msg.position.z, np.rad2deg(yaw))
        )
        self.motion_manager.approach(
            msg.position.x, msg.position.y, msg.position.z, yaw
        )
