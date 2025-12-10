#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import (
    Empty,
    Float64,
    Int64,
)  # High Level Commander のコマンド用メッセージ
from geometry_msgs.msg import Pose, Vector3, Quaternion


# 実際のCrazyflieクライアント (cf_client2.py) のインターフェースを模倣
class MockCfClient:
    def __init__(self, uri, cf_logger, drone_id):
        # cflibの初期化は行わない
        rospy.loginfo(f"[CF{drone_id}] Initializing Mock Client.")
        self.drone_id = drone_id
        self.mocap_topic_name = (
            f"mocap_node/mocap/flapper{drone_id}/pose"  # 本来は購読用
        )

        # High-Level Commander の指令をシミュレーターへ送るPublisherを定義
        self.takeoff_pub = rospy.Publisher(
            f"cf{drone_id}/sim/takeoff_cmd", Float64, queue_size=1
        )
        self.land_pub = rospy.Publisher(
            f"cf{drone_id}/sim/land_cmd", Float64, queue_size=1
        )
        self.goto_pub = rospy.Publisher(
            f"cf{drone_id}/sim/goto_cmd", Pose, queue_size=1
        )
        self.stop_pub = rospy.Publisher(
            f"cf{drone_id}/sim/stop_cmd", Empty, queue_size=1
        )

        # --- cflib のコールバックを模倣する処理 ---
        # 接続完了を待つ代わりに、ダミーのコールバックをすぐに実行
        rospy.sleep(0.5)  # ROSトピックの準備を待つ
        self.fully_connected_callback(uri)
        rospy.loginfo(f"[CF{self.drone_id}] Fully connected (Simulated)")

    def fully_connected_callback(self, uri):
        # ログの設定、Kalman/PIDの有効化などはスキップ（シミュレーションなので）

        # mocap_sub は不要（シミュレーター側で姿勢を決定するため）
        # ただし、cf_client2.pyのコードが self.mocap_sub を参照している場合は、
        # ダミーのSubscriberを定義してエラーを回避する必要があるかもしれません。
        # ここではエラー回避のため、ダミーのSubscriberを定義しておきます。
        self.mocap_sub = rospy.Subscriber(
            self.mocap_topic_name, PoseStamped, self.mocap_sub_callback
        )

    def mocap_sub_callback(self, msg):
        # 本来はCrazyflieにMoCapを送信するが、シミュレーションでは何もしない
        pass

    def takeoff(self, height=1, duration=3):
        # High Level Commander の代わりにROSトピックで指令をシミュレーターに送る
        self.takeoff_pub.publish(Float64(height))

    def land(self, height=0, duration=3, yaw=0.0):
        # High Level Commander の代わりにROSトピックで指令をシミュレーターに送る
        self.land_pub.publish(Float64(height))

    def stop(self):
        # High Level Commander の代わりにROSトピックで指令をシミュレーターに送る
        self.stop_pub.publish(Empty())

    def go_to(self, x, y, z, yaw, duration_s, relative=False):
        # High Level Commander の代わりにROSトピックで指令をシミュレーターに送る
        pose_cmd = Pose(Vector3(x, y, z), Quaternion(0, 0, 0, 1))  # Yawは一旦無視
        self.goto_pub.publish(pose_cmd)

    def close(self):
        rospy.loginfo(f"[CF{self.drone_id}] Mock Client closed.")

    # パラメータ設定系のメソッドはそのまま残すが、中身は何もしない (No-op)
    def adjust_orientation_sensitivity(self, orientation_std_dev=0.06):
        pass

    def activate_kalman_estimator(self):
        pass

    def activate_pid_controller(self):
        pass

    def tune_pid_gains(self):
        pass
