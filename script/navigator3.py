#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Quaternion
from std_msgs.msg import Empty, Float64, String
from scipy.spatial.transform import Rotation as R

class Navigator():
    def __init__(self, theta_scale=0.1, r_min=0.8, r_max=2.5, eye_h=1.5):
        # 軌道設計パラメータ
        self.theta_scale = theta_scale
        self.r_min = r_min
        self.r_max = r_max
        self.eye_h = eye_h
        
        # --- 定点待機位置の設定 ---
        self.takeoff_x = 0.0  # 待機場所のX
        self.takeoff_y = 0.0  # 待機場所のY
        self.takeoff_z = 2.3  # 待機場所のZ（離陸直後の高度）

        # 状態管理
        self.phase = 'takeoff_hover'  # 最初のフェーズ
        self.running = False

        # --- Subscribers ---
        # ドローン、胸、手の位置姿勢(MoCapデータ)を取得
        self.drone_pose_sub = rospy.Subscriber('mocap_node/mocap/flapper/pose', PoseStamped, self.drone_cb)
        self.chest_pose_sub = rospy.Subscriber('mocap_node/mocap/chest/pose', PoseStamped, self.chest_cb)
        self.hand_pose_sub = rospy.Subscriber('mocap_node/mocap/hand/pose', PoseStamped, self.hand_cb)
        
        # 制御の開始・停止指示を受け取る
        self.approach_start_sub = rospy.Subscriber('approach_start', Empty, self.start_cb)
        self.approach_stop_sub = rospy.Subscriber('approach_stop', Empty, self.stop_cb)

        # --- Publishers ---
        self.approach_pub = rospy.Publisher('approach', Pose, queue_size=1)
        self.distance_chest_drone_on_XY_plane_pub = rospy.Publisher('distance/chest_drone_on_XY_plane', Float64, queue_size=1)
        self.distance_chest_hand_on_XY_plane_pub = rospy.Publisher('distance/chest_hand_on_XY_plane', Float64, queue_size=1)
        self.distance_hand_drone_on_XY_plane_pub = rospy.Publisher('distance/hand_drone_on_XY_plane', Float64, queue_size=1)
        
        self.phase_pub = rospy.Publisher('navigator/phase', String, queue_size=1)
        self.ps_radius_pub = rospy.Publisher('navigator/ps_radius', Float64, queue_size=1)

        # 内部キャッシュ
        self.drone_raw_pose = None
        self.drone_p = None
        self.chest_p = None
        self.hand_p = None

    # --- Callback functions ---
    def drone_cb(self, msg):
        self.drone_raw_pose = msg.pose
        self.drone_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def chest_cb(self, msg):
        self.chest_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def hand_cb(self, msg):
        self.hand_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def start_cb(self, msg):
        if not self.running:
            self.run()

    def stop_cb(self, msg):
        self.running = False
        rospy.loginfo("Approach stop command received.")

    # --- Math & Logic functions ---

    # 高さに応じたパーソナルスペースの半径を取得
    def get_radius(self, z):
        return self.r_min + (self.r_max - self.r_min) / (1 + np.exp(-20 * (z - self.eye_h)))

    # 胸の方を向くクォータニオンを取得
    def look_at_quaternion(self):
        if self.drone_raw_pose is None or self.chest_p is None:
            return Quaternion(0, 0, 0, 1)
        vector_r_p = self.chest_p[:2] - self.drone_p[:2]
        target_theta = np.arctan2(vector_r_p[1], vector_r_p[0])
        drone_ori = self.drone_raw_pose.drone_orientation
        current_theta = R.from_quat([drone_ori.x, drone_ori.y, drone_ori.z, drone_ori.w]).as_euler('xyz')[2]
        diff = (target_theta - current_theta + np.pi) % (2 * np.pi) - np.pi
        goal_theta = current_theta + diff * self.theta_scale
        q = R.from_euler('xyz', [0, 0, goal_theta]).as_quat()
        return Quaternion(q[0], q[1], q[2], q[3])

    # --- メイン制御ループ ---
    def run(self):
        rospy.loginfo('Navigator loop started at takeoff_hover')
        self.running = True
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and self.running:
            if any(p is None for p in [self.drone_p, self.chest_p, self.hand_p]):
                rospy.logwarn_throttle(2, "Waiting for all mocap poses...")
                rate.sleep()
                continue

            # 現在のパーソナルスペース半径を取得
            r_curr = self.get_radius(self.drone_p[2])
            # xy平面内の各距離を計算
            dist_c_d = np.linalg.norm(self.drone_p[:2] - self.chest_p[:2])
            dist_c_h = np.linalg.norm(self.hand_p[:2] - self.chest_p[:2])
            dist_h_d = np.linalg.norm(self.drone_p[:2] - self.hand_p[:2])
            dist_to_hand_z = abs(self.drone_p[2] - (self.hand_p[2] + 0.3))

            # デバッグ・モニタリング用に距離をパブリッシュ
            self.distance_chest_drone_on_XY_plane_pub.publish(dist_c_d)
            self.distance_chest_hand_on_XY_plane_pub.publish(dist_c_h)
            self.distance_hand_drone_on_XY_plane_pub.publish(dist_h_d)
            self.ps_radius_pub.publish(r_curr)
            self.phase_pub.publish(self.phase)

            # --- フェーズ遷移ロジック ---
            if self.phase == 'takeoff_hover':
                # 指定した定点(3次元)との距離を計算
                dist_to_takeoff = np.linalg.norm(self.drone_p - np.array([self.takeoff_x, self.takeoff_y, self.takeoff_z]))
                if dist_to_takeoff < 0.2: # 20cm以内に近づいたら次へ
                    self.phase = 'preparing'
                    rospy.loginfo("Reached takeoff station point. Switching to preparing phase.")
            
            if self.phase == 'preparing':
                if abs(self.drone_p[2] - 3.0) < 0.3 and dist_c_d >= (r_curr - 0.1):
                    self.phase = 'leading'
            
            if self.phase == 'leading':
                if dist_c_d < r_curr + 0.1:
                    self.phase = 'circling'
            
            if self.phase == 'circling' or self.phase == 'leading':
                if dist_h_d < 0.15 and dist_to_hand_z < 0.1:
                    self.phase = 'docked'

            # --- 移動制御ベクトル計算 ---
            v_move = np.array([0.0, 0.0])
            goal_z = self.drone_p[2]

            if self.phase == 'takeoff_hover':
                # 定点を目指す移動
                v_move = np.array([self.takeoff_x, self.takeoff_y]) - self.drone_p[:2]
                goal_z = self.takeoff_z

            elif self.phase == 'preparing':
                v_c_d_vec = self.drone_p[:2] - self.chest_p[:2]
                target_r_pos = (v_c_d_vec / (dist_c_d + 1e-5)) * (r_curr + 0.5)
                v_move = target_r_pos - v_c_d_vec
                goal_z = 3.0

            elif self.phase == 'leading':
                v_move = self.hand_p[:2] - self.drone_p[:2]
                goal_z = self.hand_p[2] + 0.3

            elif self.phase == 'circling':
                goal_z = self.hand_p[2] + 0.3
                attraction = np.clip(dist_h_d / 0.5, 0.2, 1.0)
                theta_rot = 0.08
                rot_mat = np.array([[np.cos(theta_rot), -np.sin(theta_rot)], 
                                    [np.sin(theta_rot), np.cos(theta_rot)]])
                v_c_d_vec = self.drone_p[:2] - self.chest_p[:2]
                r_target = dist_c_h
                r_next = max(dist_c_d + (r_target - dist_c_d) * 0.1, r_curr + 0.05)
                v_next_pos = (rot_mat @ (v_c_d_vec / dist_c_d)) * r_next
                v_move = ((self.chest_p[:2] + v_next_pos) - self.drone_p[:2]) * attraction
                if dist_h_d < 0.5:
                    v_pull = (self.hand_p[:2] - self.drone_p[:2]) * (1.0 - attraction)
                    v_move += v_pull

            else: # docked
                v_move = np.array([0.0, 0.0])
                goal_z = self.hand_p[2] + 0.3

            # 指令値の正規化（最大0.1m制限）
            move_norm = np.linalg.norm(v_move)
            if move_norm > 0.1:
                v_move = (v_move / move_norm) * 0.1

            goal_pos = Vector3(self.drone_p[0] + v_move[0], 
                               self.drone_p[1] + v_move[1], 
                               goal_z)
            
            self.approach_pub.publish(Pose(goal_pos, self.look_at_quaternion()))
            rate.sleep()

        self.running = False
        rospy.loginfo('drone stay')

if __name__ == '__main__':
    try:
        rospy.init_node('navigator')
        nav = Navigator(theta_scale=0.1)
        rospy.loginfo("Navigator Node Ready.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
