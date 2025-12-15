#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from script.shared.variables import *
from scipy.spatial.transform import Rotation as R
import numpy as np
from script.shared.util import extract_yaw


class CommandRouter:
    # 2つのMotionManagerを引数に追加
    def __init__(self, mm1, mm2):
        self.mm1 = mm1
        self.mm2 = mm2

        # 共通コマンドの購読（robot.pyのMultiFlapperがパブリッシュ）
        self.takeoff_sub = rospy.Subscriber(
            "/teleop_command/takeoff", Empty, self.takeoff_sub_callback
        )
        self.land_sub = rospy.Subscriber(
            "/teleop_command/land", Empty, self.land_sub_callback
        )
        self.stop_sub = rospy.Subscriber(
            "/teleop_command/halt", Empty, self.stop_sub_callback
        )
        self.palm_land_sub = rospy.Subscriber(
            "/teleop_command/palm_land", Pose, self.palm_land_sub_callback
        )

        # 個別アプローチ指令の購読（Navigator1, Navigator2がパブリッシュ）
        self.approach_sub1 = rospy.Subscriber(
            "/flapper1/cmd_pose", Pose, self.approach_sub_callback1
        )
        self.approach_sub2 = rospy.Subscriber(
            "/flapper2/cmd_pose", Pose, self.approach_sub_callback2
        )

    def takeoff_sub_callback(self, msg):
        self.mm1.takeoff()
        self.mm2.takeoff()

    def land_sub_callback(self, msg):
        self.mm1.land()
        self.mm2.land()

    def stop_sub_callback(self, msg):
        self.mm1.stop()
        self.mm2.stop()

    def palm_land_sub_callback(self, msg):
        yaw = extract_yaw(msg.orientation)
        rospy.loginfo("Palm land command received with yaw: %f" % np.rad2deg(yaw))
        # 2機に一斉に指令を分配
        self.mm1.palm_land(yaw)
        self.mm2.palm_land(yaw)

    def approach_sub_callback1(self, msg):
        yaw = extract_yaw(msg.orientation)
        rospy.loginfo(
            "[CF1] Approach to x: %f, y: %f, z: %f, yaw: %f"
            % (msg.position.x, msg.position.y, msg.position.z, np.rad2deg(yaw))
        )
        self.mm1.approach(msg.position.x, msg.position.y, msg.position.z, yaw)

    def approach_sub_callback2(self, msg):
        yaw = extract_yaw(msg.orientation)
        rospy.loginfo(
            "[CF2] Approach to x: %f, y: %f, z: %f, yaw: %f"
            % (msg.position.x, msg.position.y, msg.position.z, np.rad2deg(yaw))
        )
        self.mm2.approach(msg.position.x, msg.position.y, msg.position.z, yaw)
