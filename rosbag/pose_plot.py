#!/usr/bin/env python
import argparse
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32

# `/state` の値に対応するラベル
STATE_LABELS = {
    0: "START",
    1: "TAKEOFF",
    2: "LAND",
    3: "STAY1",
    4: "APPROACH1",
    5: "PALM_LAND",
    6: "STOP"
}

def visualize_pose_data(bag_path, actual_topic, target_topic, state_topic, chest_topic, hand_topic):
    # データ格納用リスト
    actual_x, actual_y, actual_z, actual_t = [], [], [], []
    target_x, target_y, target_z, target_t = [], [], [], []
    chest_x, chest_y, chest_z, chest_t = [], [], [], []
    hand_x, hand_y, hand_z, hand_t = [], [], [], []
    actual_quivers, target_quivers = [], []
    state_labels = []
    special_labels = []  # Stay, Approach のラベル

    time_interval = 0.5  # 0.3 秒間隔でプロット
    last_actual_time = None
    last_target_time = None
    last_chest_time = None
    last_hand_time = None
    last_state = None
    plotting = False  # プロットのON/OFFフラグ
    chest_hand_plotting = False
    arrow_length = 0.1  # 矢印の長さ
    line_break_threshold = 1.0  # 1秒以上空いた場合に線を切る
    last_target_point = None  # Approachラベル用
    last_actual_point = None  # Approachラベル用

    # rosbags のデータを読み込み
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages():
            current_time = t.to_sec()

            # /state トピックを監視
            if topic == state_topic:
                state_value = msg.data
                if state_value != last_state:
                    state_labels.append((current_time, state_value))  # ラベル用のデータを保存
                    last_state = state_value

                if state_value == 1:
                    plotting = True  # プロット開始
                elif state_value == 3:
                    chest_hand_plotting = True
                elif state_value == 6:
                    plotting = False  # プロット終了
                    chest_hand_plotting = False
                    break  # 6になったら処理を終了

            # プロットが有効でない間はスキップ
            if not plotting:
                continue

            if topic == target_topic:
                if last_target_time is None or (current_time - last_target_time) >= time_interval:
                    last_target_time = current_time
                    x, y, z = msg.position.x, msg.position.y, msg.position.z
                    target_x.append(x)
                    target_y.append(y)
                    target_z.append(z)
                    target_t.append(current_time)

                    # 目標姿勢の回転を取得
                    qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
                    rot_matrix = tf_trans.quaternion_matrix([qx, qy, qz, qw])[:3, :3]

                    # 矢印の方向（X軸）
                    direction = rot_matrix @ np.array([arrow_length, 0, 0])
                    target_quivers.append(direction)

                    # Approach ラベル
                    if last_target_point and (current_time - last_target_point[0]) > line_break_threshold:
                        special_labels.append((last_target_point[0:], "STAY2"))
                        special_labels.append(([current_time, x, y, z], "APPROACH2"))

                    last_target_point = (current_time, x, y, z)

            elif topic == actual_topic:
                if last_actual_time is None or (current_time - last_actual_time) >= time_interval:
                    last_actual_time = current_time
                    x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
                    actual_x.append(x)
                    actual_y.append(y)
                    actual_z.append(z)
                    actual_t.append(current_time)

                    # クォータニオンから回転行列を取得
                    qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
                    rot_matrix = tf_trans.quaternion_matrix([qx, qy, qz, qw])[:3, :3]

                    # 矢印の方向（X軸）
                    direction = rot_matrix @ np.array([arrow_length, 0, 0])
                    actual_quivers.append(direction)
                    

            elif topic == chest_topic and chest_hand_plotting:
                if last_chest_time is None or (current_time - last_chest_time) >= time_interval:
                    last_chest_time = current_time
                    x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
                    chest_x.append(x)
                    chest_y.append(y)
                    chest_z.append(z)
                    chest_t.append(current_time)
            
            elif topic == hand_topic and chest_hand_plotting:
                if last_hand_time is None or (current_time - last_hand_time) >= time_interval:
                    last_hand_time = current_time
                    x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
                    hand_x.append(x)
                    hand_y.append(y)
                    hand_z.append(z)
                    hand_t.append(current_time)

    # 3Dプロットの作成
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 目標位置
    ax.scatter(target_x, target_y, target_z, c='b', marker='x', label="Target Position", s=10)
    for i in range(len(target_x) - 1):
        if target_t[i + 1] - target_t[i] < line_break_threshold:
            ax.plot([target_x[i], target_x[i + 1]], [target_y[i], target_y[i + 1]], [target_z[i], target_z[i + 1]], c='b', linestyle='-', linewidth=1)

    # 実際の位置
    ax.scatter(actual_x, actual_y, actual_z, c='r', marker='o', label="Actual Position", s=10)
    for i in range(len(actual_x) - 1):
        if actual_t[i + 1] - actual_t[i] < line_break_threshold:
            ax.plot([actual_x[i], actual_x[i + 1]], [actual_y[i], actual_y[i + 1]], [actual_z[i], actual_z[i + 1]], c='r', linestyle='-', linewidth=1)

    # Chest の位置
    ax.scatter(chest_x, chest_y, chest_z, c='g', marker='o', label="Chest Position", s=10)
    for i in range(len(chest_x) - 1):
        if chest_t[i + 1] - chest_t[i] < line_break_threshold:
            ax.plot([chest_x[i], chest_x[i + 1]], [chest_y[i], chest_y[i + 1]], [chest_z[i], chest_z[i + 1]], c='g', linestyle='-', linewidth=1)

    # Hand の位置
    ax.scatter(hand_x, hand_y, hand_z, c='y', marker='o', label="Hand Position", s=10)
    for i in range(len(hand_x) - 1):
        if hand_t[i + 1] - hand_t[i] < line_break_threshold:
            ax.plot([hand_x[i], hand_x[i + 1]], [hand_y[i], hand_y[i + 1]], [hand_z[i], hand_z[i + 1]], c='y', linestyle='-', linewidth=1)

    # 姿勢の矢印
    for i in range(len(actual_x)):
        ax.quiver(actual_x[i], actual_y[i], actual_z[i], *actual_quivers[i], color='r', length=arrow_length, normalize=True, arrow_length_ratio=0.5)
    for i in range(len(target_x)):
        ax.quiver(target_x[i], target_y[i], target_z[i], *target_quivers[i], color='b', length=arrow_length, normalize=True, arrow_length_ratio=0.5)

    # `/state` のラベル
    for t_val, state_val in state_labels:
        if state_val in STATE_LABELS:
            closest_index = min(range(len(actual_t)), key=lambda i: abs(actual_t[i] - t_val))
            label_pos_for_actual = (actual_x[closest_index], actual_y[closest_index], actual_z[closest_index] + (-0.9 if STATE_LABELS[state_val] == "STOP" else -0.3 if STATE_LABELS[state_val] == "STAY1" else 0.5 if STATE_LABELS[state_val] == "APPROACH1" else 0.9 if STATE_LABELS[state_val] == "PALM_LAND" else 0.3))
            ax.text(*label_pos_for_actual, STATE_LABELS[state_val], color="red", fontsize=10, fontweight="bold")
            ax.plot([actual_x[closest_index], label_pos_for_actual[0]], [actual_y[closest_index], label_pos_for_actual[1]], [actual_z[closest_index], label_pos_for_actual[2]], "k--")

    # Stay / Approach ラベル
    for (t, x, y, z), label in special_labels:
        label_pos_for_actual = (x, y, z + (0.3 if label == "STAY2" else -0.9))
        ax.text(*label_pos_for_actual, label, color="blue", fontsize=10, fontweight="bold")
        ax.plot([x, label_pos_for_actual[0]], [y, label_pos_for_actual[1]], [z, label_pos_for_actual[2]], "black", linestyle="dashed")
        # special_labels に最も近い時刻での実際の位置にもラベルを付ける
        closest_index = min(range(len(actual_t)), key=lambda i: abs(actual_t[i] - t))
        label_pos_for_actual = (actual_x[closest_index], actual_y[closest_index], actual_z[closest_index] + (0.7 if label == "STAY2" else -0.5))
        ax.text(*label_pos_for_actual, label, color="red", fontsize=10, fontweight="bold")
        ax.plot([actual_x[closest_index], label_pos_for_actual[0]], [actual_y[closest_index], label_pos_for_actual[1]], [actual_z[closest_index], label_pos_for_actual[2]], "k--")
        # special_labels に最も近い時刻での手と胸の位置にもラベルを付ける
        closest_index = min(range(len(chest_t)), key=lambda i: abs(chest_t[i] - t))
        label_pos_for_chest = (chest_x[closest_index], chest_y[closest_index], chest_z[closest_index] - (-0.5 if label == "STAY2" else 0.9))
        ax.text(*label_pos_for_chest, label, color="green", fontsize=10, fontweight="bold")
        ax.plot([chest_x[closest_index], label_pos_for_chest[0]], [chest_y[closest_index], label_pos_for_chest[1]], [chest_z[closest_index], label_pos_for_chest[2]], "k--")
        closest_index = min(range(len(hand_t)), key=lambda i: abs(hand_t[i] - t))
        label_pos_for_hand = (hand_x[closest_index], hand_y[closest_index], hand_z[closest_index] - (0.5 if label == "STAY2" else 0.9))
        ax.text(*label_pos_for_hand, label, color="brown", fontsize=10, fontweight="bold")
        ax.plot([hand_x[closest_index], label_pos_for_hand[0]], [hand_y[closest_index], label_pos_for_hand[1]], [hand_z[closest_index], label_pos_for_hand[2]], "k--")

    ax.set_xlabel("X", fontweight="bold")
    ax.set_ylabel("Y", fontweight="bold")
    ax.set_zlabel("Z", fontweight="bold")
    ax.set_title("", fontweight="bold")
    ax.legend(fontsize=12, loc='best', frameon=True, prop={'weight': 'bold'})
    ax.tick_params(axis='both', labelsize=12, width=2)
    for label in ax.get_xticklabels() + ax.get_yticklabels() + ax.get_zticklabels():
        label.set_fontweight('bold')

    # 目標位置と現在位置

    # グラフの表示
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize Actual and Target Positions from ROS bag, filtered by /state, with orientation arrows and state labels.")
    bag_path = "2025-03-19-13-54-48.bag"
    actual_topic = "/mocap_node/mocap/flapper/pose"
    target_topic = "/approach"
    state_topic = "/state"
    chest_topic = "/mocap_node/mocap/chest/pose"
    hand_topic = "/mocap_node/mocap/hand/pose"

    args = parser.parse_args()
    visualize_pose_data(bag_path, actual_topic, target_topic, state_topic, chest_topic, hand_topic)