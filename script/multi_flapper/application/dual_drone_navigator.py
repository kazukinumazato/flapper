#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Pose, PoseStamped, Quaternion
from std_msgs.msg import Empty, Float64, String, Int64
from scipy.spatial.transform import Rotation as R
import threading


class NavigatorDual:
    def __init__(self, my_id, other_id, theta_scale=0.1, drone_between_dist_thresh=1.0):

        # パラメータ
        self.my_id = my_id
        self.r_min, self.r_max, self.chest2eye_h = 0.7, 1.5, 0.2
        self.eye_h = 1.5
        self.theta_scale = theta_scale
        self.drone_between_dist_thresh = drone_between_dist_thresh

        # 状態管理
        self.phase = "takeoff_hover"
        self.running = False

        # --- Subscribers ---
        # 自身の位置・胸の位置・自身が担当する手の位置
        rospy.Subscriber(
            f"mocap_node/mocap/flapper{my_id}/pose", PoseStamped, self.drone_cb
        )
        rospy.Subscriber("mocap_node/mocap/chest/pose", PoseStamped, self.chest_cb)
        rospy.Subscriber(
            f"mocap_node/mocap/hand{my_id}/pose", PoseStamped, self.hand_cb
        )

        # 【重要】相手ドローンと手の位置を購読 (衝突回避用)
        rospy.Subscriber(
            f"mocap_node/mocap/flapper{other_id}/pose", PoseStamped, self.other_drone_cb
        )
        rospy.Subscriber(
            f"mocap_node/mocap/hand{other_id}/pose", PoseStamped, self.other_hand_cb
        )

        # 制御コマンド
        rospy.Subscriber("approach_start", Empty, self.start_cb)

        # --- Publishers ---
        self.approach_pub = rospy.Publisher(
            f"flapper{my_id}/cmd_pose", Pose, queue_size=1
        )
        self.phase_pub = rospy.Publisher(f"navigator{my_id}/phase", Int64, queue_size=1)

        # キャッシュ
        self.drone_raw_pose = None
        self.drone_p = np.array([0.0, 0.0, 0.0])
        self.other_drone_p = np.array([0.0, 0.0, 0.0])
        self.chest_p = np.array([0.0, 0.0, 0.0])
        self.hand_p = np.array([0.0, 0.0, 0.0])

        # --- 定点待機位置の設定 ---
        self.takeoff_x = 0.5  # 待機場所のX
        self.takeoff_y = (self.my_id * 2 - 3) * 1.5  # 待機場所のY
        self.takeoff_z = 2.3  # 待機場所のZ（離陸直後の高度）

    # --- Callbacks ---
    def drone_cb(self, msg):
        self.drone_raw_pose = msg.pose
        self.drone_p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

    def other_drone_cb(self, msg):
        self.other_drone_p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

    def other_hand_cb(self, msg):
        self.other_hand_p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

    def chest_cb(self, msg):
        self.chest_p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )
        self.eye_h = chest_p[2] + self.chest2eye_h

    def hand_cb(self, msg):
        self.hand_p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

    def start_cb(self, msg):
        if not self.running:
            # 制御ループを別スレッドで開始
            t = threading.Thread(target=self.run)
            t.daemon = True
            t.start()

    def get_radius(self, z):
        return self.r_min + (self.r_max - self.r_min) / (
            1 + np.exp(-30 * (z - self.eye_h))
        )

    # 胸の方を向くクォータニオンを取得
    def look_at_quaternion(self):
        if self.drone_raw_pose is None or self.chest_p is None:
            return Quaternion(0, 0, 0, 1)
        vector_r_p = self.chest_p[:2] - self.drone_p[:2]
        target_theta = np.arctan2(vector_r_p[1], vector_r_p[0])
        drone_ori = self.drone_raw_pose.orientation
        current_theta = R.from_quat(
            [drone_ori.x, drone_ori.y, drone_ori.z, drone_ori.w]
        ).as_euler("xyz")[2]
        diff = (target_theta - current_theta + np.pi) % (2 * np.pi) - np.pi
        goal_theta = current_theta + diff * self.theta_scale
        q = R.from_euler("xyz", [0, 0, goal_theta]).as_quat()
        return Quaternion(q[0], q[1], q[2], q[3])

    # --- Main Loop ---
    def run(self):
        rospy.loginfo(f"Navigator {self.my_id} started")
        self.running = True
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and self.running:
            # Note: 初期データチェックは、現在のNumPy配列初期化ではp is None判定は効かないが、今回はこのまま維持する
            if any(
                p is None
                for p in [self.drone_p, self.other_drone_p, self.chest_p, self.hand_p]
            ):
                rospy.logwarn("Waiting for all position data...")
                rate.sleep()
                continue

            # --- 現在の高さでの許容距離計算 ---
            r_curr = self.get_radius(self.drone_p[2])
            # --- 距離計算(xy平面) ---
            dist_c_d = np.linalg.norm(self.drone_p[:2] - self.chest_p[:2])
            dist_h_d = np.linalg.norm(self.drone_p[:2] - self.hand_p[:2])
            dist_c_h = np.linalg.norm(self.chest_p[:2] - self.hand_p[:2])
            dist_h1_h2 = np.linalg.norm(self.hand_p[:2] - self.other_hand_p[:2])
            # --- z軸距離計算 ---
            dist_to_hand_z = abs(self.drone_p[2] - (self.hand_p[2] + 0.3))

            # --- フェーズ遷移 ---
            #  ホバリング-> 準備(遠いなら近くまで行く)->誘導->周回->ドッキング
            #                                            ->ドッキング
            if self.phase == "takeoff_hover":
                # 指定した定点(3次元)との距離を計算
                # ユーザーの意図に基づき、現在位置のXY座標をキープし、Z=takeoff_zを目指す
                dist_to_takeoff = np.linalg.norm(
                    self.drone_p
                    - np.array([self.drone_p[0], self.drone_p[1], self.takeoff_z])
                )
                rospy.loginfo(
                    f"Distance to takeoff Z: {abs(self.drone_p[2] - self.takeoff_z):.2f} m"
                )
                rospy.loginfo(
                    f"goal position: ({self.drone_p[0]:.2f}, {self.drone_p[1]:.2f}, {self.takeoff_z})"
                )
                rospy.loginfo(
                    f"current position: ({self.drone_p[0]:.2f}, {self.drone_p[1]:.2f}, {self.drone_p[2]:.2f})"
                )
                if (
                    abs(self.drone_p[2] - self.takeoff_z) < 0.2
                ):  # Z軸の差が20cm以内なら次へ
                    self.phase = "preparing"
                    rospy.loginfo(
                        "Reached to beginning height. Switching to preparing phase."
                    )
            if self.phase == "preparing":
                if self.drone_p[2] > 2.7 and dist_c_d >= r_curr - 0.1:
                    self.phase = "leading"
                    rospy.loginfo("Switching to leading phase.")
            if self.phase == "leading":
                if dist_c_d < r_curr + 0.1:
                    self.phase = "circling"
                    rospy.loginfo("Switching to circling phase.")

            if self.phase == "circling":
                if dist_c_d > r_curr + 0.3:
                    self.phase = "leading"
                    rospy.loginfo(
                        "Too far from personal space. Switching to leading phase."
                    )

            if self.phase in ["leading", "circling"]:
                if dist_h_d < 0.15 and dist_to_hand_z < 0.1:
                    self.phase = "docked"
                    rospy.loginfo("Docked to hand. Switching to docked phase.")

            # --- 移動計算 ---
            v_move = np.array([0.0, 0.0])
            goal_z = self.drone_p[2]

            phase_num = 0
            if self.phase == "takeoff_hover":
                phase_num = 1
            elif self.phase == "preparing":
                phase_num = 2
            elif self.phase == "leading":
                phase_num = 3
            elif self.phase == "circling":
                phase_num = 4
            elif self.phase == "docked":
                phase_num = 5

            if (
                dist_c_h < 0.5 or dist_h1_h2 < 0.5
            ):  # 手同士や胸と手が近すぎる場合は移動しない
                v_move = np.array([0.0, 0.0])
                self.phase_pub.publish(-phase_num)
            else:
                if self.phase == "takeoff_hover":
                    # 定点ホバリング中はXY移動なし、Z軸のみ上昇
                    v_move = np.array([0.0, 0.0])
                    goal_z = self.takeoff_z
                elif self.phase == "preparing":
                    v_c_d = self.drone_p[:2] - self.chest_p[:2]
                    target_r_pos = (v_c_d / (dist_c_d + 1e-5)) * (r_curr + 0.5)
                    v_move = target_r_pos - v_c_d
                    goal_z = 3.0
                elif self.phase == "leading":
                    v_move = self.hand_p[:2] - self.drone_p[:2]
                    # 手の高さに徐々に近づける
                    goal_z = (self.drone_p[2] * 8 + (self.hand_p[2] + 0.3) * 2) / 10
                elif self.phase == "circling":
                    # 回転軌道計算
                    theta = 0.08
                    rot_mat = np.array(
                        [
                            [np.cos(theta), -np.sin(theta)],
                            [np.sin(theta), np.cos(theta)],
                        ]
                    )
                    v_c_d = self.drone_p[:2] - self.chest_p[:2]
                    r_target = np.linalg.norm(self.hand_p[:2] - self.chest_p[:2])
                    r_next = max(dist_c_d + (r_target - dist_c_d) * 0.1, r_curr + 0.05)
                    v_next_pos = (rot_mat @ (v_c_d / dist_c_d)) * r_next
                    v_move = (self.chest_p[:2] + v_next_pos) - self.drone_p[:2]
                    # 手への引き込み
                    if dist_h_d < 0.5:
                        v_move += (self.hand_p[:2] - self.drone_p[:2]) * 0.3
                    goal_z = self.hand_p[2] + 0.3
                elif self.phase == "docked":
                    v_move = self.hand_p[:2] - self.drone_p[:2]
                    goal_z = self.hand_p[2] + 0.3
                else:
                    rospy.logwarn(f"Unknown phase: {self.phase}")

            # --- 衝突回避 ---
            dist_between = np.linalg.norm(self.drone_p[:2] - self.other_drone_p[:2])
            # ドッキング中は反発を弱める
            repulsion_weight = 0.80 if self.phase != "docked" else 0.02
            # 手の近くでは回避より目標優先
            repulsion_influence = 1.0 if dist_h_d > 0.2 else 0.0

            if dist_between < self.drone_between_dist_thresh:
                repulsion_vec = (self.drone_p[:2] - self.other_drone_p[:2]) / (
                    dist_between + 1e-5
                )
                strength = np.clip(
                    (self.drone_between_dist_thresh - dist_between) / 1.0, 0, 1
                )  # 近さに応じて反発を強く
                v_move += (
                    repulsion_vec * strength * repulsion_weight * repulsion_influence
                )

            # 目標までの距離の制限(最大1.0m)
            move_norm = np.linalg.norm(v_move)
            if move_norm > 1.0:
                v_move = (v_move / move_norm) * 1.0

            # 送信
            goal_pos = Vector3(
                self.drone_p[0] + v_move[0], self.drone_p[1] + v_move[1], goal_z
            )
            self.approach_pub.publish(
                Pose(goal_pos, self.look_at_quaternion())
            )  # 向きは簡易化
            self.phase_pub.publish(self.phase)
            rate.sleep()


if __name__ == "__main__":
    # 引数などで自身のIDと相手のIDを指定して起動
    my_id = rospy.get_param("~drone_id", 1)
    rospy.init_node(f"navigator_dual{my_id}")
    other_id = 2 if my_id == 1 else 1
    nav = NavigatorDual(my_id, other_id)
    rospy.spin()
