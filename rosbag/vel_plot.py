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
vel_topic = "/log/state_estimate/vel"  # Vector3型
drone_topic = "/mocap_node/mocap/flapper/pose"  # PoseStamped
chest_topic = "/mocap_node/mocap/chest/pose"  # PoseStamped
hand_topic = "/mocap_node/mocap/hand/pose"  # PoseStamped

# 状態管理
recording = False  # 記録開始フラグ
vel_data = {
    "time": [], "vx": [], "vy": [], "vz": [], "speed_xy": []
}

drone_pos = {}  # drone の位置情報
chest_pos = {}  # chest の位置情報
hand_pos = {}  # hand の位置情報

deceleration_distance_times = []  # 距離が1.0mになる時刻
close_times = []  # 距離が0.3mになる時刻
approach_time = None

with rosbag.Bag(bag_path, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=[state_topic, vel_topic, drone_topic, chest_topic, hand_topic]):
        timestamp = t.to_sec()

        # /state トピックの処理
        if topic == state_topic:
            state_value = msg.data  # std_msgs/Int32 など
            if state_value == 1:  # TAKEOFF
                recording = True 
            elif state_value == 6:  # STOP
                recording = False
            elif state_value == 4 and approach_time == None:  # APPROACH
                print(timestamp)
                approach_time = timestamp



        # /vel トピックの処理 (Vector3)
        if topic == vel_topic and recording:
            vx, vy, vz = msg.x, msg.y, msg.z
            speed_xy = np.sqrt(vx**2 + vy**2)  # 2次元速度を計算
            
            vel_data["time"].append(timestamp)
            vel_data["vx"].append(vx)
            vel_data["vy"].append(vy)
            vel_data["vz"].append(vz)
            vel_data["speed_xy"].append(speed_xy)

        # /drone トピックの処理 (PoseStamped)
        if topic == drone_topic:
            drone_pos[timestamp] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        # /chest トピックの処理 (PoseStamped)
        if topic == chest_topic:
            chest_pos[timestamp] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        if topic == hand_topic:
            hand_pos[timestamp] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

# 距離の計算と1.0mの時刻記録
times = []
times.extend(drone_pos.keys())
times.extend(chest_pos.keys())
times = sorted(times)
for t in times:
    drone_pos_t = min(drone_pos.keys(), key=lambda x: abs(x - t))
    chest_pos_t = min(chest_pos.keys(), key=lambda x: abs(x - t))
    hand_pos_t = min(hand_pos.keys(), key=lambda x: abs(x - t))
    drone_chest_dx = drone_pos[drone_pos_t][0] - chest_pos[chest_pos_t][0]
    drone_chest_dy = drone_pos[drone_pos_t][1] - chest_pos[chest_pos_t][1]
    drone_chest_distance = np.sqrt(drone_chest_dx**2 + drone_chest_dy**2)
    drone_hand_dx = drone_pos[drone_pos_t][0] - hand_pos[hand_pos_t][0]
    drone_hand_dy = drone_pos[drone_pos_t][1] - hand_pos[hand_pos_t][1]
    drone_hand_distance = np.sqrt(drone_hand_dx**2 + drone_hand_dy**2)
    if np.isclose(drone_chest_distance, 1.0, atol=0.05):  # 1.0m ± 0.05m
        deceleration_distance_times.append(t)
    elif np.isclose(drone_hand_distance, 0.3, atol=0.05):
        close_times.append(t)
    if len(deceleration_distance_times) > 0 and len(close_times) > 0:
        break

# グラフの作成
fig, axes = plt.subplots(2, 1, figsize=(10, 12), sharex=True)
axes = [None, None, axes[0], axes[1]]
labels = ["Velocity X (m/s)", "Velocity Y (m/s)", "Velocity Z (m/s)", "Speed XY (m/s)"]
colors = ["r", "g", "b", "purple"]
keys = ["vx", "vy", "vz", "speed_xy"]
for i in range(4):
    if i == 0 or i == 1:
        continue
    axes[i].plot(vel_data["time"], vel_data[keys[i]], color=colors[i], linewidth=2)
    axes[i].set_ylabel(labels[i], fontsize=18, fontweight="bold")
    axes[i].tick_params(axis="both", labelsize=16, width=2)
    for label in axes[i].get_xticklabels() + axes[i].get_yticklabels():
        label.set_fontweight("bold")
    
    # 距離1.0mの時刻で縦線を引く
    axes[i].axvline(x=deceleration_distance_times[0], color='black', linestyle='--', linewidth=1.5, label="Drone_hand_distance = 1.0m")
    axes[i].axvline(x=close_times[0], color='blue', linestyle='--', linewidth=1.5, label="Drone_hand_distance = 0.3m")
    axes[i].axvline(x=approach_time, color='red', linestyle='--', linewidth=1.5, label="Approach_beginning")
    
axes[3].set_xlabel("Time (s)", fontsize=18, fontweight="bold")
# 凡例（太字 & グラフ外に配置）
ax = axes[2]
legend = ax.legend(fontsize=18, loc="upper right", bbox_to_anchor=(1.1, 1))
for text in legend.get_texts():
    text.set_fontweight("bold")

plt.suptitle("", fontsize=16, fontweight="bold")
plt.tight_layout()
plt.show()
