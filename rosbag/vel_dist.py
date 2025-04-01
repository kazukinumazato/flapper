#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys

# 引数からbagファイルのパスを取得
if len(sys.argv) < 2:
    print("使用方法: python script.py <bagファイルのパス>")
    sys.exit(1)

bag_path = "2025-03-19-12-59-23-change-dist_close-to-0dot3-andnd-altitude-to-0dot2-simple-hand-land-success.bag"

# 使用するトピック
state_topic = "/state"
drone_topic = "/mocap_node/mocap/flapper/pose"
hand_topic = "/mocap_node/mocap/hand/pose"
vel_topic = "/log/state_estimate/vel"  # Vector3型

# 状態管理
recording = False  # 記録開始フラグ
drone_data = {}  # timestamp -> (x, y)
hand_data = {}  # timestamp -> (x, y)
vel_data = {}  # timestamp -> (vx, vy)

distance_vs_velocity = {
    "distance": [],
    "xy_velocity": []
}

with rosbag.Bag(bag_path, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=[state_topic, drone_topic, hand_topic, vel_topic]):
        timestamp = t.to_sec()

        # /state トピックの処理
        if topic == state_topic:
            state_value = msg.data  # std_msgs/Int32 など
            if state_value == 1:  # TAKEOFF
                recording = True
            elif state_value == 6:  # STOP
                recording = False

        # /drone_pos トピックの処理 (PoseStamped)
        if topic == drone_topic and recording:
            drone_data[timestamp] = (msg.pose.position.x, msg.pose.position.y)

        # /hand_pos トピックの処理 (PoseStamped)
        if topic == hand_topic and recording:
            hand_data[timestamp] = (msg.pose.position.x, msg.pose.position.y)

        # /vel トピックの処理 (Vector3)
        if topic == vel_topic and recording:
            vel_data[timestamp] = (msg.x, msg.y)  # XY 速度のみ

# 近いタイムスタンプ同士で距離とXY速度を計算
drone_times = sorted(drone_data.keys())
hand_times = sorted(hand_data.keys())
vel_times = sorted(vel_data.keys())

for t_drone in drone_times:
    # handとvelのデータで最も近いタイムスタンプを探す
    t_hand = min(hand_times, key=lambda t: abs(t - t_drone))
    t_vel = min(vel_times, key=lambda t: abs(t - t_drone))

    pos_drone = np.array(drone_data[t_drone])
    pos_hand = np.array(hand_data[t_hand])
    vx, vy = vel_data[t_vel]

    # XY 平面での距離を計算
    dist_drone_hand = np.linalg.norm(pos_drone - pos_hand)

    # XY 平面の速度を計算
    xy_velocity = np.sqrt(vx**2 + vy**2)

    # 結果を保存
    distance_vs_velocity["distance"].append(dist_drone_hand)
    distance_vs_velocity["xy_velocity"].append(xy_velocity)

# グラフの作成
plt.figure(figsize=(8, 6))

# 散布図プロット
plt.scatter(distance_vs_velocity["distance"], distance_vs_velocity["xy_velocity"], color="blue", alpha=0.6, label="XY Velocity vs Drone-Hand Distance")

# 軸ラベルとタイトル
plt.xlabel("Drone-Hand XY Distance (m)", fontsize=14, fontweight="bold")
plt.ylabel("XY Velocity (m/s)", fontsize=14, fontweight="bold")
plt.title("XY Velocity vs Drone-Hand Distance", fontsize=16, fontweight="bold")

# 凡例（太字）
legend = plt.legend(fontsize=12)
for text in legend.get_texts():
    text.set_fontweight("bold")

# 軸の目盛りを太字に
plt.tick_params(axis="both", labelsize=12, width=2)
for label in plt.gca().get_xticklabels() + plt.gca().get_yticklabels():
    label.set_fontweight("bold")

plt.grid()
plt.show()
