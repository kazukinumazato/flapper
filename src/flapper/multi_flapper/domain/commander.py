#!usr/bin/env python
import rospy
from std_msgs.msg import Empty, Int64, Float64
from geometry_msgs.msg import Pose
from flapper.shared.variables import *


class MultiFlapperCommander:
    def __init__(self):
        self.state_sub = rospy.Subscriber("/cf1/state", Int64, self.state1_callback)
        self.state_sub2 = rospy.Subscriber("/cf2/state", Int64, self.state2_callback)
        self.takeoff_pub = rospy.Publisher("/teleop_command/takeoff", Empty)
        self.land_pub = rospy.Publisher("/teleop_command/land", Empty)
        self.halt_pub = rospy.Publisher("/teleop_command/halt", Empty)

        self.palm_land_pub = rospy.Publisher("/palm_land", Float64)

        self.state1 = RobotState.START
        self.state2 = RobotState.START

    # --- コールバック ---
    def state1_callback(self, msg):
        self.state1 = msg.data

    def state2_callback(self, msg):
        self.state2 = msg.data

    # --- 状態判定ロジック (state.pyから呼ばれる) ---
    def are_both_in_state(self, target_state):
        """両機が指定した状態(HOVER等)ならTrue"""
        return (self.state1 == target_state) and (self.state2 == target_state)

    def is_any_in_state(self, target_state):
        """どちらか1機でも指定した状態ならTrue"""
        return (self.state1 == target_state) or (self.state2 == target_state)

    # --- コマンド実行 (一斉指示) ---
    def takeoff(self):
        rospy.loginfo("Sending takeoff command to both drones")
        self.takeoff_pub.publish(Empty())

    def land(self):
        rospy.loginfo("Sending land command to both drones")
        self.land_pub.publish(Empty())

    def stop(self):
        rospy.loginfo("Sending emergency halt to both drones")
        self.halt_pub.publish(Empty())

    def palm_land(self, yaw):
        rospy.loginfo("Sending palm land command to both drones")
        # Yaw角のみを Pose.orientation.z に入れて送信するなどの取り決めが必要
        # pose_msg = Pose()
        # pose_msg.orientation.z = yaw  # 簡易的にYaw角を運ぶ
        self.palm_land_pub.publish(yaw)
