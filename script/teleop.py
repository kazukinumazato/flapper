#!/usr/bin/env python   

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from variables import *

# 入力受付クラス
class Teleop():
    def __init__(self, navigator):
        self.navigator = navigator
        self.takeoff_sub = rospy.Subscriber('teleop_command/takeoff', Empty, self.takeoff_sub_callback)
        self.land_sub = rospy.Subscriber('teleop_command/land', Empty, self.land_sub_callback)
        self.approach_sub = rospy.Subscriber('teleop_command/approach', Empty, self.approach_sub_callback)
        self.palm_land_sub = rospy.Subscriber('teleop_command/palm_land', Empty, self.palm_land_sub_callback)
        self.stop_sub = rospy.Subscriber('teleop_command/halt', Empty, self.stop_sub_callback)

    def takeoff_sub_callback(self, msg):
        self.navigator.takeoff()

    def land_sub_callback(self, msg):
        self.navigator.land()

    def approach_sub_callback(self, msg):
        self.navigator.approach()

    def palm_land_sub_callback(self, msg):
        self.navigator.palm_land()

    def stop_sub_callback(self, msg):
        self.navigator.stop()
