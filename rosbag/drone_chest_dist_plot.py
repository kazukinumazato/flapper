#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys

# 引数からbagファイルのパスを取得
if len(sys.argv) < 2:
    print("使用方法: python script.py <bagファイルのパス>")
    sys.exit(1)

bag_path = sys.argv[1]

# 使用するトピック
state_topic = "/state"
drone_topic = "/mocap_node/mocap/flapper/pose"
chest_topic = "/mocap_node/mocap/chest/pose"
hand_topic = "/mocap_node/mocap/hand/pose"

# 状態管理
recording = False  # 記録開始フラグ
drone_data = {}  # timestamp -> (x, y)
chest_data = {}  # timestamp -> (x, y)
hand_data = {}  # timestamp -> (x, y)
distances = {
    "time": [],
    "chest_hand": [],
    "drone_chest": [],
    "drone_hand": []
}

with rosbag.Bag(bag_path, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=[state_topic, drone_topic, chest_topic, hand_topic]):
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

        # /chest_pos トピックの処理 (PoseStamped)
        if topic == chest_topic and recording:
            chest_data[timestamp] = (msg.pose.position.x, msg.pose.position.y)

        # /hand_pos トピックの処理 (PoseStamped)
        if topic == hand_topic and recording:
            hand_data[timestamp] = (msg.pose.position.x, msg.pose.position.y)

# 近いタイムスタンプ同士で距離を計算
drone_times = sorted(drone_data.keys())
chest_times = sorted(chest_data.keys())
hand_times = sorted(hand_data.keys())

for t_drone in drone_times:
    # chest, handのデータで最も近いタイムスタンプを探す
    t_chest = min(chest_times, key=lambda t: abs(t - t_drone))
    t_hand = min(hand_times, key=lambda t: abs(t - t_drone))

    pos_drone = np.array(drone_data[t_drone])
    pos_chest = np.array(chest_data[t_chest])
    pos_hand = np.array(hand_data[t_hand])

    # 各距離を計算 (XY平面)
    dist_chest_hand = np.linalg.norm(pos_chest - pos_hand)
    dist_drone_chest = np.linalg.norm(pos_drone - pos_chest)
    dist_drone_hand = np.linalg.norm(pos_drone - pos_hand)

    # 結果を保存
    distances["time"].append(t_drone)
    distances["chest_hand"].append(dist_chest_hand)
    distances["drone_chest"].append(dist_drone_chest)
    distances["drone_hand"].append(dist_drone_hand)

# グラフの作成
plt.figure(figsize=(10, 6))

# 距離ごとのプロット
plt.plot(distances["time"], distances["chest_hand"], color="r", linewidth=2, label="Chest-Hand Distance")
plt.plot(distances["time"], distances["drone_chest"], color="g", linewidth=2, label="Drone-Chest Distance")
plt.plot(distances["time"], distances["drone_hand"], color="b", linewidth=2, label="Drone-Hand Distance")
print("minimum drone-chest distance: ", min(distances["drone_chest"]))
print("maximum chest-hand distance: ", max(distances["chest_hand"]))

# しきい値 1.0m の横線
plt.axhline(y=1.0, color="black", linestyle="dotted", linewidth=2, label="Decerelation_radius = 1.0m")
plt.axhline(y=0.3, color="blue", linestyle="dotted", linewidth=2, label="Dist_close = 0.3m")

# 軸ラベルとタイトル
plt.xlabel("Time (s)", fontsize=14, fontweight="bold")
plt.ylabel("XY Distance (m)", fontsize=14, fontweight="bold")
plt.title("", fontsize=16, fontweight="bold")

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
