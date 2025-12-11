#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Navigator3用MoCapシミュレータ
- ドローンの動力学をシミュレート
- 人間（胸・手）の動きを生成
"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion, Point
from std_msgs.msg import String
import numpy as np
import math


class MoCapSimulatorNav:
    def __init__(self):
        rospy.loginfo("[MOCAP_SIM_NAV] Initializing MoCap Simulator")
        
        # ドローン状態
        self.drone_pos = np.array([2.0, 2.0, 3.0])
        self.drone_quat = np.array([0.0, 0.0, 0.0, 1.0])
        # 初期ターゲットは「現在のドローン位置」に設定（勝手に原点へ移動しないように）
        self.target_pos = self.drone_pos.copy()
        self.target_quat = self.drone_quat.copy()
        
        # 人間状態 - navigator3 の期待値に合わせる
        # takeoff_x=0.0, takeoff_y=0.0 を中心に、胸は固定
        self.chest_pos = np.array([0.0, 0.0, 1.5])  # 固定
        self.hand_pos = np.array([0.3, 0.0, 1.0])   # 初期位置：胸から30cm離れた場所
        
        # パブリッシャー
        self.flapper_pub = rospy.Publisher(
            'mocap_node/mocap/flapper/pose', PoseStamped, queue_size=1
        )
        self.chest_pub = rospy.Publisher(
            'mocap_node/mocap/chest/pose', PoseStamped, queue_size=1
        )
        self.hand_pub = rospy.Publisher(
            'mocap_node/mocap/hand/pose', PoseStamped, queue_size=1
        )
        
        # サブスクライバー
        rospy.Subscriber('drone/sim/goto_cmd', Pose, self.goto_callback)
        rospy.Subscriber('navigator/phase', String, self.phase_callback)
        
        self.current_phase = "unknown"
        self.start_time = rospy.get_time()
        self.max_speed = 1.0  # m/s
        
        # タイマー (100Hz)
        rospy.Timer(rospy.Duration(0.01), self.update_and_publish)
        
        rospy.loginfo("[MOCAP_SIM_NAV] Ready")
    
    def goto_callback(self, msg):
        """目標位置を受信"""
        self.target_pos = np.array([
            msg.position.x, msg.position.y, msg.position.z
        ])
        self.target_quat = np.array([
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        ])
    
    def phase_callback(self, msg):
        """フェーズ情報を受信"""
        self.current_phase = msg.data
    
    def update_drone(self):
        """ドローンを目標に向けて移動"""
        diff = self.target_pos - self.drone_pos
        distance = np.linalg.norm(diff)
        
        if distance > 0.001:
            velocity = min(self.max_speed, distance / 0.01) * 0.01
            movement = (diff / distance) * velocity
            
            if np.linalg.norm(movement) > distance:
                self.drone_pos = self.target_pos.copy()
            else:
                self.drone_pos += movement
        
        # クォータニオン補間
        dot = np.dot(self.drone_quat, self.target_quat)
        if dot < 0:
            target_q = -self.target_quat
        else:
            target_q = self.target_quat
        
        blend_rate = min(0.05, 0.01 * 0.5)
        self.drone_quat = (1 - blend_rate) * self.drone_quat + blend_rate * target_q
        norm = np.linalg.norm(self.drone_quat)
        if norm > 0:
            self.drone_quat /= norm
    
    def update_human(self):
        """人間の動きを生成"""
        elapsed = rospy.get_time() - self.start_time
        t = elapsed
        
        # 胸: 固定位置（chest_pos は変更しない）
        # self.chest_pos = [0.0, 0.0, 1.5]  # 固定
        
        # 手: 胸の周りをゆっくり旋回
        # navigator3 の dist_h_d < 0.15 という条件に合わせるため、
        # 手が胸から 0.3m 程度の距離で動く
        hand_radius = 0.8 + 0.05 * math.sin(0.5 * t)
        hand_theta = t * 0.2  # 回転速度を遅くする
        
        self.hand_pos[0] = self.chest_pos[0] + hand_radius * math.cos(hand_theta)
        self.hand_pos[1] = self.chest_pos[1] + hand_radius * math.sin(hand_theta)
        self.hand_pos[2] = 1.0 + 0.05 * math.sin(0.7 * t)
    
    def publish_poses(self):
        """MoCap トピックをパブリッシュ"""
        timestamp = rospy.Time.now()
        
        # Flapper
        flapper_msg = PoseStamped()
        flapper_msg.header.stamp = timestamp
        flapper_msg.header.frame_id = "map"
        flapper_msg.pose.position = Point(*self.drone_pos)
        flapper_msg.pose.orientation = Quaternion(*self.drone_quat)
        self.flapper_pub.publish(flapper_msg)
        
        # Chest (固定)
        chest_msg = PoseStamped()
        chest_msg.header.stamp = timestamp
        chest_msg.header.frame_id = "map"
        chest_msg.pose.position = Point(*self.chest_pos)
        chest_msg.pose.orientation = Quaternion(0, 0, 0, 1)
        self.chest_pub.publish(chest_msg)
        
        # Hand (動き)
        hand_msg = PoseStamped()
        hand_msg.header.stamp = timestamp
        hand_msg.header.frame_id = "map"
        hand_msg.pose.position = Point(*self.hand_pos)
        hand_msg.pose.orientation = Quaternion(0, 0, 0, 1)
        self.hand_pub.publish(hand_msg)
    
    def update_and_publish(self, event):
        """メインループ"""
        self.update_drone()
        self.update_human()
        self.publish_poses()


def main():
    rospy.init_node('mocap_simulator_nav', anonymous=False)
    simulator = MoCapSimulatorNav()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass