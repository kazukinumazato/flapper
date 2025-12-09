#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Quaternion
from std_msgs.msg import Empty, Float64, String
from scipy.spatial.transform import Rotation as R

class NavigatorDual():
    def __init__(self, my_id, other_id, theta_scale=0.1):
        # パラメータ
        self.my_id = my_id
        self.r_min, self.r_max, self.eye_h = 0.8, 1.5, 1.6
        self.theta_scale = theta_scale

        # 状態管理
        self.phase = 'preparing'
        self.running = False

        # --- Subscribers ---
        # 自身の位置・胸の位置・自身が担当する手の位置
        rospy.Subscriber(f'flapper{my_id}/pose', PoseStamped, self.drone_cb)
        rospy.Subscriber('chest/pose', PoseStamped, self.chest_cb)
        rospy.Subscriber(f'hand{my_id}/pose', PoseStamped, self.hand_cb)
        
        # 【重要】相手ドローンの位置を購読 (衝突回避用)
        rospy.Subscriber(f'flapper{other_id}/pose', PoseStamped, self.other_drone_cb)

        # 制御コマンド
        rospy.Subscriber('approach_start', Empty, self.start_cb)

        # --- Publishers ---
        self.approach_pub = rospy.Publisher(f'flapper{my_id}/cmd_pose', Pose, queue_size=1)
        self.phase_pub = rospy.Publisher(f'navigator{my_id}/phase', String, queue_size=1)

        # キャッシュ
        self.drone_p = None
        self.other_drone_p = None
        self.chest_p = None
        self.hand_p = None

    def drone_cb(self, msg):
        self.drone_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    
    def other_drone_cb(self, msg):
        self.other_drone_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def chest_cb(self, msg):
        self.chest_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def hand_cb(self, msg):
        self.hand_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def start_cb(self, msg):
        if not self.running: self.run()

    def get_radius(self, z):
        return self.r_min + (self.r_max - self.r_min) / (1 + np.exp(-30 * (z - self.eye_h)))

    def run(self):
        rospy.loginfo(f'Navigator {self.my_id} started')
        self.running = True
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and self.running:
            if any(p is None for p in [self.drone_p, self.other_drone_p, self.chest_p, self.hand_p]):
                rate.sleep()
                continue

            r_curr = self.get_radius(self.drone_p[2])
            dist_c_d = np.linalg.norm(self.drone_p[:2] - self.chest_p[:2])
            dist_h_d = np.linalg.norm(self.drone_p[:2] - self.hand_p[:2])
            dist_to_hand_z = abs(self.drone_p[2] - (self.hand_p[2] + 0.3))

            # --- フェーズ遷移 (test6.py 準拠) ---
            if self.phase == 'preparing':
                if self.drone_p[2] > 2.7 and dist_c_d >= r_curr - 0.1:
                    self.phase = 'leading'
            elif self.phase == 'leading':
                if dist_c_d < r_curr + 0.1:
                    self.phase = 'circling'
            
            if self.phase in ['leading', 'circling']:
                if dist_h_d < 0.15 and dist_to_hand_z < 0.1:
                    self.phase = 'docked'

            # --- 移動計算 ---
            v_move = np.array([0.0, 0.0])
            goal_z = self.drone_p[2]

            if self.phase == 'preparing':
                v_c_d = self.drone_p[:2] - self.chest_p[:2]
                target_r_pos = (v_c_d / (dist_c_d + 1e-5)) * (r_curr + 0.5)
                v_move = target_r_pos - v_c_d
                goal_z = 3.0
            elif self.phase == 'leading':
                v_move = self.hand_p[:2] - self.drone_p[:2]
                goal_z = self.hand_p[2] + 0.3
            elif self.phase == 'circling':
                # 回転軌道計算
                theta = 0.08
                rot_mat = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
                v_c_d = self.drone_p[:2] - self.chest_p[:2]
                r_target = np.linalg.norm(self.hand_p[:2] - self.chest_p[:2])
                r_next = max(dist_c_d + (r_target - dist_c_d) * 0.1, r_curr + 0.05)
                v_next_pos = (rot_mat @ (v_c_d / dist_c_d)) * r_next
                v_move = (self.chest_p[:2] + v_next_pos) - self.drone_p[:2]
                # 手への引き込み
                if dist_h_d < 0.5:
                    v_move += (self.hand_p[:2] - self.drone_p[:2]) * 0.3
                goal_z = self.hand_p[2] + 0.3
            elif self.phase == 'docked':
                v_move = self.hand_p[:2] - self.drone_p[:2]
                goal_z = self.hand_p[2] + 0.3

            # --- 【test6コアロジック】衝突回避 ---
            dist_between = np.linalg.norm(self.drone_p[:2] - self.other_drone_p[:2])
            # ドッキング中は反発を弱める
            repulsion_weight = 0.15 if self.phase != 'docked' else 0.02
            # 手の近くでは回避より目標優先
            repulsion_influence = 1.0 if dist_h_d > 0.2 else 0.0

            if dist_between < 0.7:
                repulsion_vec = (self.drone_p[:2] - self.other_drone_p[:2]) / (dist_between + 1e-5)
                strength = np.clip((0.7 - dist_between) / 0.7, 0, 1)
                v_move += repulsion_vec * strength * repulsion_weight * repulsion_influence

            # 移動量の制限
            move_norm = np.linalg.norm(v_move)
            if move_norm > 0.1:
                v_move = (v_move / move_norm) * 0.1

            # 送信
            goal_pos = Vector3(self.drone_p[0] + v_move[0], self.drone_p[1] + v_move[1], goal_z)
            self.approach_pub.publish(Pose(goal_pos, Quaternion(0,0,0,1))) # 向きは簡易化
            self.phase_pub.publish(self.phase)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('navigator_dual')
    # 引数などで自身のIDと相手のIDを指定して起動
    my_id = rospy.get_param('~drone_id', 1) 
    other_id = 2 if my_id == 1 else 1
    nav = NavigatorDual(my_id, other_id)
    rospy.spin()
