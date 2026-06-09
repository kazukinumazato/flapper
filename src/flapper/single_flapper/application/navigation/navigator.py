#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Vector3, PoseStamped, Pose, Quaternion
from std_msgs.msg import Empty, Float64
from flapper.shared.util import dist
from flapper.shared.variables import *
import numpy as np
from scipy.spatial.transform import Rotation as R


class Navigator:
    """
    ドローンの自律移動を制御するクラス。
    人間の「胸(chest)」と「手(hand)」の位置関係に基づき、
    ドローンの目標位置と姿勢(常に胸の方を向く)を算出・パブリッシュします。
    """

    def __init__(self, deceleration_radius=1.0, dist_close=0.3):

        # --- Subscribers ---
        # ドローン、胸、手の位置姿勢(MoCapデータ)を取得
        self.drone_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/flapper/pose", PoseStamped, self.drone_pose_sub_callback
        )
        self.chest_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/chest/pose", PoseStamped, self.chest_pose_sub_callback
        )
        self.hand_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/hand/pose", PoseStamped, self.hand_pose_sub_callback
        )

        # 制御の開始・停止指示を受け取る
        self.approach_start_sub = rospy.Subscriber(
            "approach_start", Empty, self.approach_start_sub_callback
        )
        self.approach_stop_sub = rospy.Subscriber(
            "approach_stop", Empty, self.approach_stop_sub_callback
        )

        # --- Publishers ---
        self.approach_pub = rospy.Publisher("approach", Pose)  # 目標ポーズ
        self.distance_chest_drone_on_XY_plane_pub = rospy.Publisher(
            "distance/chest_drone_on_XY_plane", Float64
        )
        self.distance_chest_hand_on_XY_plane_pub = rospy.Publisher(
            "distance/chest_hand_on_XY_plane", Float64
        )
        self.distance_hand_drone_on_XY_plane_pub = rospy.Publisher(
            "distance/hand_drone_on_XY_plane", Float64
        )

        self.circular_pub = rospy.Publisher("circular", Float64)  # 円周運動フラグ

        # --- 制御パラメータの設定 ---
        self.deceleration_radius = deceleration_radius  # 減速を開始する半径
        self.dist_close = dist_close  # 「至近距離」と見なす閾値
        self.running = False  # 制御実行フラグ

    # --- callback functions ---
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

    # --- メイン制御ループ ---
    def run(self, timeout=120):
        rospy.loginfo("approach started")
        self.running = True
        start_t = rospy.get_time()

        while rospy.get_time() <= start_t + timeout and self.running:
            chest_pos = self.chest_pose.position
            hand_pos = self.hand_pose.position
            drone_pos = self.drone_pose.position

            # 平面上の各点間距離の計算
            distance_chest_drone_on_XY_plane = dist(chest_pos, drone_pos, True)
            distance_chest_hand_on_XY_plane = dist(chest_pos, hand_pos, True)
            distance_hand_drone_on_XY_plane = dist(hand_pos, drone_pos, True)

            # デバッグ・モニタリング用に距離をパブリッシュ
            self.distance_chest_drone_on_XY_plane_pub.publish(
                Float64(distance_chest_drone_on_XY_plane)
            )
            self.distance_chest_hand_on_XY_plane_pub.publish(
                Float64(distance_chest_hand_on_XY_plane)
            )
            self.distance_hand_drone_on_XY_plane_pub.publish(
                Float64(distance_hand_drone_on_XY_plane)
            )

            # --- Z軸（高度）の制御計算 ---
            height_above_hand = 0.3  # 手の30cm上を目標とする
            scale = 0.3  # 移動速度のスケーリング係数

            # 手の高さに基づいて目標高度を算出 (P制御に近い形)
            goal_dist_z = (hand_pos.z + height_above_hand - drone_pos.z) * scale
            goal_pos_z = hand_pos.z + height_above_hand - goal_dist_z

            # --- 姿勢の制御計算 ---
            # ドローンの正面を常に胸の方向に向けるクォータニオンを取得
            goal_ori = self.look_at_quaternion(drone_pos, chest_pos)

            # --- XY平面上の動作判定 ---
            # 条件1: ドローンが胸に近すぎる場合 (胸からの距離 < 手からの距離)
            if distance_chest_drone_on_XY_plane < distance_chest_hand_on_XY_plane:
                self.circular_pub.publish(1)
                rospy.loginfo("drone is running on circle.")

                # 胸を中心に、手がある方向へ円周上に目標点を設定
                vector_c_h = np.array(
                    [hand_pos.x - chest_pos.x, hand_pos.y - chest_pos.y]
                )  # 胸→手
                vector_c_d = np.array(
                    [drone_pos.x - chest_pos.x, drone_pos.y - chest_pos.y]
                )  # 胸→ドローン

                # 外積と内積から現在の角度の差を求め、目標角度を算出
                cross = np.cross(vector_c_h, vector_c_d)
                dot = np.dot(vector_c_h, vector_c_d)
                goal_theta = np.arctan2(cross, dot) * scale

                # 回転行列を用いて円周上の目標位置を生成
                r_matrix = np.array(
                    [
                        [np.cos(goal_theta), -np.sin(goal_theta)],
                        [np.sin(goal_theta), np.cos(goal_theta)],
                    ]
                )
                vector_c_g = r_matrix @ vector_c_h

                goal_pos = Vector3(
                    chest_pos.x + vector_c_g[0], chest_pos.y + vector_c_g[1], goal_pos_z
                )
                self.approach_pub.publish(Pose(goal_pos, goal_ori))

            # 条件2: 胸からは十分離れている場合
            else:
                self.circular_pub.publish(0)
                # 条件2-A: 手の付近（減速圏内）にいる場合
                if distance_chest_drone_on_XY_plane < self.deceleration_radius:
                    # スムーズな接近のため目標位置を補間
                    goal_dist_x = (hand_pos.x - drone_pos.x) * scale
                    goal_dist_y = (hand_pos.y - drone_pos.y) * scale

                    goal_pos = Vector3(
                        hand_pos.x - goal_dist_x, hand_pos.y - goal_dist_y, goal_pos_z
                    )
                    self.approach_pub.publish(Pose(goal_pos, goal_ori))

                # 条件2-B: 手からかなり遠い場合（全速で接近圏内へ移動）
                else:
                    rospy.loginfo("outside of deceleration_radius")
                    vector_d_h = np.array(
                        [hand_pos.x - drone_pos.x, hand_pos.y - drone_pos.y]
                    )  # ドローン→手
                    vector_d_h_normalized = vector_d_h / np.linalg.norm(
                        vector_d_h
                    )  # 単位ベクトル化

                    # 減速圏の境界線付近を目標値として直進移動
                    goal_pos = Vector3(
                        drone_pos.x
                        + vector_d_h_normalized[0]
                        * self.deceleration_radius
                        * (1 - scale),
                        drone_pos.y
                        + vector_d_h_normalized[1]
                        * self.deceleration_radius
                        * (1 - scale),
                        goal_pos_z,
                    )
                    self.approach_pub.publish(Pose(goal_pos, goal_ori))

            rospy.sleep(0.1)  # 10Hz周期で更新

        self.running = False
        rospy.loginfo("drone stay")

    def look_at_quaternion(self, r, p, theta_scale=0.1):
        """
        ドローン(r)から胸(p)への方位角を計算し、Yaw軸を補間したクォータニオンを返す
        """
        # 胸へのベクトルを計算
        vector_r_p = np.array([p.x - r.x, p.y - r.y])
        e_x = np.array([1, 0])  # 基準となるX軸

        # 胸への目標角度を算出
        cross = np.cross(e_x, vector_r_p)
        dot = np.dot(e_x, vector_r_p)
        chest_theta = np.arctan2(cross, dot)

        # 現在のドローンのYaw角を取得
        drone_ori = self.drone_pose.orientation
        current_theta = R.from_quat(
            [drone_ori.x, drone_ori.y, drone_ori.z, drone_ori.w]
        ).as_euler("xyz")[2]

        # 急な回転を防ぐため現在の角度と目標角度の間を線形補間
        goal_theta = (chest_theta - current_theta) * theta_scale + current_theta

        # オイラー角からクォータニオンへ変換
        quaternion = R.from_euler("xyz", [0, 0, goal_theta]).as_quat()
        return Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])


if __name__ == "__main__":
    rospy.init_node("navigator")
    Navigator()
    rospy.spin()
