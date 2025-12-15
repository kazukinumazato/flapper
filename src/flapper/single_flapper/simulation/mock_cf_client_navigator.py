#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Navigator3用のモッククライアント
- navigator3.py から approach トピックを購読
- ドローンシミュレータに指令を送信
"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion
from std_msgs.msg import Float64, Empty
import numpy as np


class MockCfClientNavigator:
    """navigator3 用モッククライアント"""
    
    def __init__(self):
        rospy.loginfo("[MOCK_CF_NAV] Initializing Mock CF Client for Navigator3")
        
        # シミュレータへのパブリッシャー
        self.takeoff_pub = rospy.Publisher(
            'drone/sim/takeoff_cmd', Float64, queue_size=1
        )
        self.land_pub = rospy.Publisher(
            'drone/sim/land_cmd', Float64, queue_size=1
        )
        self.goto_pub = rospy.Publisher(
            'drone/sim/goto_cmd', Pose, queue_size=1
        )
        
        # navigator3 からの指令を購読
        self.approach_sub = rospy.Subscriber(
            'approach', Pose, self.approach_callback
        )
        
        rospy.loginfo("[MOCK_CF_NAV] Ready to receive approach commands")
    
    def approach_callback(self, msg):
        """navigator3 からの approach トピックを受信して、シミュレータに転送"""
        # goto_cmd として転送
        self.goto_pub.publish(msg)
    
    def takeoff(self, height=1, duration=3):
        self.takeoff_pub.publish(Float64(height))
        rospy.loginfo(f"[MOCK_CF_NAV] Takeoff command: height={height}")
    
    def land(self, height=0, duration=3, yaw=0.0):
        self.land_pub.publish(Float64(height))
        rospy.loginfo(f"[MOCK_CF_NAV] Land command: height={height}")
    
    def stop(self):
        pass
    
    def go_to(self, x, y, z, yaw, duration_s, relative=False):
        pose_cmd = Pose(Vector3(x, y, z), Quaternion(0, 0, 0, 1))
        self.goto_pub.publish(pose_cmd)
    
    def close(self):
        rospy.loginfo("[MOCK_CF_NAV] Mock client closed")
    
    def activate_kalman_estimator(self):
        pass
    
    def activate_pid_controller(self):
        pass
    
    def adjust_orientation_sensitivity(self, orientation_std_dev=0.06):
        pass
    
    def tune_pid_gains(self):
        pass