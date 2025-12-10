#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Quaternion
from std_msgs.msg import Empty, Float64, String
from scipy.spatial.transform import Rotation as R


class Navigator:
    def __init__(self, theta_scale=0.1, r_min=0.8, r_max=2.5, eye_h=1.5):
        # 軌道設計パラメータ
        self.theta_scale = theta_scale
        self.r_min = r_min
        self.r_max = r_max
        self.eye_h = eye_h

        # 状態管理
        self.phase = "preparing"
        self.running = False

        # --- Subscribers ---
        # Mocapデータ
        self.drone_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/flapper/pose", PoseStamped, self.drone_cb
        )
        self.chest_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/chest/pose", PoseStamped, self.chest_cb
        )
        self.hand_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/hand/pose", PoseStamped, self.hand_cb
        )

        # 制御コマンド
        self.approach_start_sub = rospy.Subscriber(
            "approach_start", Empty, self.start_cb
        )
        self.approach_stop_sub = rospy.Subscriber("approach_stop", Empty, self.stop_cb)

        # --- Publishers (制御系) ---
        self.approach_pub = rospy.Publisher("approach", Pose, queue_size=1)

        # --- Publishers (モニタリング用トピック：元のコードと同一) ---
        self.distance_chest_drone_on_XY_plane_pub = rospy.Publisher(
            "distance/chest_drone_on_XY_plane", Float64, queue_size=1
        )
        self.distance_chest_hand_on_XY_plane_pub = rospy.Publisher(
            "distance/chest_hand_on_XY_plane", Float64, queue_size=1
        )
        self.distance_hand_drone_on_XY_plane_pub = rospy.Publisher(
            "distance/hand_drone_on_XY_plane", Float64, queue_size=1
        )

        # --- Publishers (新規デバッグ用) ---
        self.phase_pub = rospy.Publisher("navigator/phase", String, queue_size=1)
        self.ps_radius_pub = rospy.Publisher(
            "navigator/ps_radius", Float64, queue_size=1
        )

        # 内部キャッシュ
        self.drone_raw_pose = None
        self.drone_p = np.array([0.0, 0.0, 0.0])
        self.chest_p = np.array([0.0, 0.0, 0.0])
        self.hand_p = np.array([0.0, 0.0, 0.0])

    # --- Callback functions ---
    def drone_cb(self, msg):
        self.drone_raw_pose = msg.pose
        self.drone_p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

    def chest_cb(self, msg):
        self.chest_p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

    def hand_cb(self, msg):
        self.hand_p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

    def start_cb(self, msg):
        if not self.running:
            self.run()

    def stop_cb(self, msg):
        self.running = False
        rospy.loginfo("Approach stop command received.")

    # --- Math & Logic functions ---
    def get_radius(self, z):
        """高度に応じてパーソナルスペースの半径をシグモイド曲線で計算"""
        return self.r_min + (self.r_max - self.r_min) / (
            1 + np.exp(-20 * (z - self.eye_h))
        )

    def look_at_quaternion(self):
        if self.drone_raw_pose is None or self.chest_p is None:
            return Quaternion(0, 0, 0, 1)

        # 胸(Chest)の方角を目標にする
        vec_r_p = self.chest_p[:2] - self.drone_p[:2]
        target_theta = np.arctan2(vec_r_p[1], vec_r_p[0])

        # 現在のドローンのヨー角
        ori = self.drone_raw_pose.orientation
        current_theta = R.from_quat([ori.x, ori.y, ori.z, ori.w]).as_euler("xyz")[2]

        # 最短経路での角度差を計算
        diff = (target_theta - current_theta + np.pi) % (2 * np.pi) - np.pi
        goal_theta = current_theta + diff * self.theta_scale

        q = R.from_euler("xyz", [0, 0, goal_theta]).as_quat()
        return Quaternion(q[0], q[1], q[2], q[3])

    def run(self):
        rospy.loginfo("PS-based navigator loop started")
        self.running = True
        rate = rospy.Rate(20)  # 20Hzで制御

        while not rospy.is_shutdown() and self.running:
            if any(p is None for p in [self.drone_p, self.chest_p, self.hand_p]):
                rospy.logwarn_throttle(2, "Waiting for all mocap poses...")
                rate.sleep()
                continue

            # 基本パラメータの計算
            r_curr = self.get_radius(self.drone_p[2])
            dist_c_d = np.linalg.norm(self.drone_p[:2] - self.chest_p[:2])
            dist_c_h = np.linalg.norm(self.hand_p[:2] - self.chest_p[:2])
            dist_h_d = np.linalg.norm(self.drone_p[:2] - self.hand_p[:2])
            dist_to_hand_z = abs(self.drone_p[2] - (self.hand_p[2] + 0.3))

            # モニタリングトピックの発行
            self.distance_chest_drone_on_XY_plane_pub.publish(dist_c_d)
            self.distance_chest_hand_on_XY_plane_pub.publish(dist_c_h)
            self.distance_hand_drone_on_XY_plane_pub.publish(dist_h_d)
            self.ps_radius_pub.publish(r_curr)
            self.phase_pub.publish(self.phase)

            # --- フェーズ遷移ロジック ---
            if self.phase == "preparing":
                # 安全高度へ上昇
                if abs(self.drone_p[2] - 3.0) < 0.3 and dist_c_d >= (r_curr - 0.1):
                    self.phase = "leading"
            elif self.phase == "leading":
                # 手の上空付近へ到達
                if dist_c_d < r_curr + 0.1:
                    self.phase = "circling"
            elif self.phase == "circling":
                # 最終ドッキング判定
                if dist_h_d < 0.15 and dist_to_hand_z < 0.1:
                    self.phase = "docked"

            # --- 移動制御ベクトル計算 ---
            v_move = np.array([0.0, 0.0])
            goal_z = self.drone_p[2]

            if self.phase == "preparing":
                # PS外側かつ高度3mを目指す
                v_c_d_vec = self.drone_p[:2] - self.chest_p[:2]
                target_r_pos = (v_c_d_vec / (dist_c_d + 1e-5)) * (r_curr + 0.5)
                v_move = target_r_pos - v_c_d_vec
                goal_z = 3.0

            elif self.phase == "leading":
                # 手のXY位置の直上へ
                v_move = self.hand_p[:2] - self.drone_p[:2]
                goal_z = self.hand_p[2] + 0.3

            elif self.phase == "circling":
                # 手の周りを回りながら接近（吸着係数）
                goal_z = self.hand_p[2] + 0.3
                attraction = np.clip(dist_h_d / 0.5, 0.2, 1.0)

                # 旋回行列
                theta_rot = 0.08
                rot_mat = np.array(
                    [
                        [np.cos(theta_rot), -np.sin(theta_rot)],
                        [np.sin(theta_rot), np.cos(theta_rot)],
                    ]
                )

                v_c_d_vec = self.drone_p[:2] - self.chest_p[:2]
                # 目標半径は手の位置の半径
                r_target = dist_c_h
                r_next = max(dist_c_d + (r_target - dist_c_d) * 0.1, r_curr + 0.05)

                # 次の目標位置を旋回成分から生成
                v_next_pos = (rot_mat @ (v_c_d_vec / dist_c_d)) * r_next
                v_move = (
                    (self.chest_p[:2] + v_next_pos) - self.drone_p[:2]
                ) * attraction

                # 接近時は直線的に引き寄せる
                if dist_h_d < 0.5:
                    v_pull = (self.hand_p[:2] - self.drone_p[:2]) * (1.0 - attraction)
                    v_move += v_pull

            else:  # docked: 位置維持
                v_move = np.array([0.0, 0.0])
                goal_z = self.hand_p[2] + 0.3

            # --- 指令値の正規化と出力 ---
            # 安全のため、1ステップの最大移動量を0.1mに制限
            move_norm = np.linalg.norm(v_move)
            if move_norm > 0.1:
                v_move = (v_move / move_norm) * 0.1

            goal_pos = Vector3(
                self.drone_p[0] + v_move[0], self.drone_p[1] + v_move[1], goal_z
            )

            self.approach_pub.publish(Pose(goal_pos, self.look_at_quaternion()))
            rate.sleep()

        self.running = False
        rospy.loginfo("navigator process ended")


# --- Main Entry Point ---
if __name__ == "__main__":
    try:
        rospy.init_node("navigator_ps_node", anonymous=True)
        # 起動時に Navigator インスタンスを生成
        nav = Navigator(theta_scale=0.1)

        rospy.loginfo("Navigator Node Ready. Pub/Sub started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
