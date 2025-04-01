#!/usr/bin/env python3
import rosbag
import numpy as np
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

# --- 設定 ---
bagfile = "2025-03-19-12-59-23-change-dist_close-to-0dot3-andnd-altitude-to-0dot2-simple-hand-land-success.bag"  # rosbag ファイル
goal_topic = "/approach"
trajectory_topic = "/mocap_node/mocap/flapper/pose"
time_offsets = np.arange(0.0, 1.6, 0.1)  # 0.1秒 ～ 1.5秒 のステップ

# --- データを格納 ---
goal_msgs = []  # (time, x, y, z)
trajectory_msgs = []  # (time, x, y, z)

# --- rosbag を開く ---
with rosbag.Bag(bagfile, "r") as bag:
    for topic, msg, t in bag.read_messages():
        if topic == goal_topic:
            goal_msgs.append((t.to_sec(), msg.position.x, msg.position.y, msg.position.z))
        elif topic == trajectory_topic:
            trajectory_msgs.append((t.to_sec(), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

# --- 各 time_offset に対する RMSE を計算 ---
rmse_values = []

for time_offset in time_offsets:
    errors = []
    for goal_time, gx, gy, gz in goal_msgs:
        target_time = goal_time + time_offset

        # 0.1秒後の trajectory を探す（最も近いもの）
        closest_traj = min(trajectory_msgs, key=lambda x: abs(x[0] - target_time))

        tx, ty, tz = closest_traj[1], closest_traj[2], closest_traj[3]
        error = np.sqrt((gx - tx) ** 2 + (gy - ty) ** 2 + (gz - tz) ** 2)
        errors.append(error)

    # --- RMSE を計算 ---
    rmse = np.sqrt(np.mean(np.array(errors) ** 2))
    rmse_values.append(rmse)
    print(f"time_offset: {time_offset:.1f} s, RMSE: {rmse:.4f} m")

# --- RMSE をプロット ---
plt.figure(figsize=(8, 5))
plt.plot(time_offsets, rmse_values, marker="o", linestyle="-", color="b", label="RMSE")
plt.xlabel("Time Offset (s)")
plt.ylabel("RMSE (m)")
plt.title("RMSE vs Time Offset")
plt.grid(True)
plt.legend()
plt.show()
