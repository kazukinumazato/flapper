#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Navigator3テスト用メインスクリプト
- モッククライアント
- MoCapシミュレータ
- Navigator3
これらを統合して実行
"""

import rospy
import sys
import os

# パスを追加
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from script.single_flapper.simulation.mock_cf_client_navigator import (
    MockCfClientNavigator,
)
from script.single_flapper.simulation.mocap_simulator_nav import MoCapSimulatorNav
from script.single_flapper.application.navigation.navigator_3d import (
    Navigator3D as Navigator,
)


def main():
    rospy.init_node("test_navigator_node", anonymous=False)

    rospy.loginfo("=" * 60)
    rospy.loginfo("Navigator3 Test with MoCap Simulation")
    rospy.loginfo("=" * 60)

    # 1. MoCapシミュレータを起動（胸・手・ドローンの位置を配信）
    rospy.loginfo("[TEST] Starting MoCap Simulator...")
    mocap_sim = MoCapSimulatorNav()
    rospy.sleep(0.5)

    # 2. モッククライアントを起動（approach トピックを受け取る）
    rospy.loginfo("[TEST] Starting Mock CF Client...")
    mock_client = MockCfClientNavigator()
    rospy.sleep(0.5)

    # 3. Navigator_3dを起動
    rospy.loginfo("[TEST] Starting Navigator3...")
    navigator = Navigator(theta_scale=0.1, r_min=0.8, r_max=2.5, chest2eye_h=0.2)
    rospy.sleep(0.5)

    rospy.loginfo("=" * 60)
    rospy.loginfo("All components started. Ready for simulation.")
    rospy.loginfo("Initial setup:")
    rospy.loginfo(f"  Drone will start at: (2.0, 2.0, 0.0)")
    rospy.loginfo(f"  Takeoff point: (0.0, 0.0, 2.3)")
    rospy.loginfo(f"  Chest (fixed): (0.0, 0.0, 1.5)")
    rospy.loginfo(f"  Hand (moving): (0.3, 0.0, 1.0) initially")
    rospy.loginfo("")
    rospy.loginfo("To start navigation, publish to /approach_start topic:")
    rospy.loginfo("  rostopic pub /approach_start std_msgs/Empty '{}'")
    rospy.loginfo("=" * 60)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

