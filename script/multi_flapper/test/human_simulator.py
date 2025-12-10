#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion
import numpy as np


class HumanSimulator:
    def __init__(self):
        # --- Publisher: Human MoCapトピック ---
        self.chest_pub = rospy.Publisher(
            "mocap_node/mocap/chest/pose", PoseStamped, queue_size=1
        )
        self.hand1_pub = rospy.Publisher(
            "mocap_node/mocap/hand1/pose", PoseStamped, queue_size=1
        )
        self.hand2_pub = rospy.Publisher(
            "mocap_node/mocap/hand2/pose", PoseStamped, queue_size=1
        )

        # --- シミュレートする静的な姿勢 ---
        # チェスト (胸: 固定位置)
        self.chest_pose = Pose(Vector3(1.0, 0.0, 1.3), Quaternion(0, 0, 0, 1))
        # ハンド1 (右手: CF1の目標)
        self.hand1_pos = np.array([1.5, -0.5, 1.0])
        self.hand1_ori = Quaternion(0, 0, 0, 1)
        # ハンド2 (左手: CF2の目標)
        self.hand2_pos = np.array([1.5, 0.5, 1.0])
        self.hand2_ori = Quaternion(0, 0, 0, 1)

        # --- 更新タイマー (30HzでMoCapデータをパブリッシュ) ---
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.publish_poses)

        rospy.loginfo("[Human SIM] Human Simulator ready.")

    def publish_poses(self, event):
        now = rospy.Time.now()

        # Hand1の動きを少しシミュレーション (例えば、ゆっくりと上下)
        t = rospy.get_time()
        self.hand1_pos[2] = 1.0 + 0.1 * np.sin(t * 0.5)

        # Chest
        self._publish(
            self.chest_pub, self.chest_pose.position, self.chest_pose.orientation, now
        )

        # Hand1
        self._publish(self.hand1_pub, Vector3(*self.hand1_pos), self.hand1_ori, now)

        # Hand2
        self._publish(self.hand2_pub, Vector3(*self.hand2_pos), self.hand2_ori, now)

    def _publish(self, pub, pos, ori, now):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position = pos
        pose_msg.pose.orientation = ori
        pub.publish(pose_msg)


def human_simulator_main():
    rospy.init_node("human_simulator_node")
    HumanSimulator()
    rospy.spin()


if __name__ == "__main__":
    human_simulator_main()
