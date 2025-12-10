#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion
from std_msgs.msg import Empty, Float64
import numpy as np


class CfSimulator:
    def __init__(
        self,
        drone_id,
        initial_pos=np.array([0.0, 0.0, 0.0]),
        initial_quat=np.array([0.0, 0.0, 0.0, 1.0]),
    ):
        self.drone_id = drone_id

        # --- ドローンの現在位置と目標位置 ---
        self.current_pos = initial_pos
        self.current_quat = initial_quat
        self.target_pos = initial_pos
        self.target_quat = initial_quat
        self.is_flying = False

        # --- Publisher: ドローンのシミュレートされたMoCap姿勢 ---
        self.mocap_pub = rospy.Publisher(
            f"mocap_node/mocap/flapper{drone_id}/pose", PoseStamped, queue_size=1
        )

        # --- Subscriber: MockCfClientからのHigh-Levelコマンド ---
        rospy.Subscriber(f"cf{drone_id}/sim/takeoff_cmd", Float64, self.takeoff_cb)
        rospy.Subscriber(f"cf{drone_id}/sim/land_cmd", Float64, self.land_cb)
        rospy.Subscriber(f"cf{drone_id}/sim/goto_cmd", Pose, self.goto_cb)
        rospy.Subscriber(f"cf{drone_id}/sim/stop_cmd", Empty, self.stop_cb)

        # --- シミュレーションタイマー (100Hzで更新) ---
        rospy.Timer(rospy.Duration(0.01), self.update_and_publish)

        rospy.loginfo(f"[SIM{drone_id}] CF Simulator ready. Initial pos: {initial_pos}")

    def takeoff_cb(self, msg):
        self.target_pos = np.array([self.current_pos[0], self.current_pos[1], msg.data])
        self.is_flying = True
        rospy.loginfo(
            f"[SIM{self.drone_id}] Takeoff command received. Target Z: {msg.data}"
        )

    def land_cb(self, msg):
        self.target_pos = np.array([self.current_pos[0], self.current_pos[1], msg.data])
        self.is_flying = False
        rospy.loginfo(
            f"[SIM{self.drone_id}] Land command received. Target Z: {msg.data}"
        )

    def goto_cb(self, msg):
        # go_toの目標位置を設定 (X, Y, Z)
        self.target_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        # Yaw角もあればここで target_quat に反映させる
        rospy.loginfo(
            f"[SIM{self.drone_id}] GoTo command received. Target: {self.target_pos}"
        )

    def stop_cb(self, msg):
        # 目標位置を現在位置に固定（緊急停止）
        self.target_pos = self.current_pos
        self.is_flying = False
        rospy.loginfo(f"[SIM{self.drone_id}] Stop command received.")

    def update_and_publish(self, event):
        # 目標位置に向けて線形補間（単純な運動シミュレーション）
        dt = 0.01

        # 移動ベクトルと距離の計算
        diff = self.target_pos - self.current_pos
        distance = np.linalg.norm(diff)

        # 速度制限 (Max 1 m/s)
        max_speed = 1.0

        if distance > 0.001:
            # 速度 v = 距離 / 予測移動時間 (durationはMockCfClientが無視したので、ここでは固定速度)
            velocity = min(max_speed * distance, max_speed) * dt  # 減速を表現

            # 移動
            movement = diff / distance * velocity

            # 目標を通り過ぎないようにクリップ
            if np.linalg.norm(movement) > distance:
                self.current_pos = self.target_pos
            else:
                self.current_pos += movement

        # --- MoCapトピックとしてパブリッシュ ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"

        pose_msg.pose.position = Vector3(
            self.current_pos[0], self.current_pos[1], self.current_pos[2]
        )

        # クォータニオンは固定 (回転シミュレーションは省略)
        pose_msg.pose.orientation = Quaternion(
            self.current_quat[0],
            self.current_quat[1],
            self.current_quat[2],
            self.current_quat[3],
        )

        self.mocap_pub.publish(pose_msg)


def cf_simulator_main():
    rospy.init_node("cf_simulator_node")

    # 1号機シミュレーター (初期位置: X=0.0, Y=0.0, Z=0.0)
    CfSimulator(drone_id=1, initial_pos=np.array([0.0, 0.0, 0.0]))
    # 2号機シミュレーター (初期位置: X=0.5, Y=0.0, Z=0.0)
    CfSimulator(drone_id=2, initial_pos=np.array([0.5, 0.0, 0.0]))

    rospy.spin()


if __name__ == "__main__":
    cf_simulator_main()
