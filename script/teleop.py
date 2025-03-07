#!/usr/bin/env python   

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from variables import *

# 入力受付クラス
class Teleop():
    def __init__(self, motion_manager):
        self.motion_manager = motion_manager
        
        self.takeoff_sub = rospy.Subscriber('teleop_command/takeoff', Empty, self.takeoff_sub_callback)
        self.land_sub = rospy.Subscriber('teleop_command/land', Empty, self.land_sub_callback)
        self.stop_sub = rospy.Subscriber('teleop_command/halt', Empty, self.stop_sub_callback)

        self.palm_land_sub = rospy.Subscriber('palm_land', Empty, self.palm_land_sub_callback)
        self.approach_sub = rospy.Subscriber('approach', Vector3, self.approach_sub_callback)

    def takeoff_sub_callback(self, msg):
        self.motion_manager.takeoff()

    def land_sub_callback(self, msg):
        self.motion_manager.land()

    def stop_sub_callback(self, msg):
        self.motion_manager.stop()
    
    def palm_land_sub_callback(self, msg):
        self.motion_manager.palm_land()

    def approach_sub_callback(self, msg):
        self.motion_manager.approach(msg.x, msg.y, msg.z)
