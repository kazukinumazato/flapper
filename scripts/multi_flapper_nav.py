#!/usr/bin/env python3
import rospy
from flapper.multi_flapper.application.dual_drone_navigator import NavigatorDual

if __name__ == "__main__":
    # 引数などで自身のIDと相手のIDを指定して起動
    rospy.init_node(f"navigator_dual")
    nav1 = NavigatorDual(1, 2, theta_scale=0.1, repulsion_gain=1.0)
    nav2 = NavigatorDual(2, 1, theta_scale=0.1, repulsion_gain=2.0)
    rospy.spin()
