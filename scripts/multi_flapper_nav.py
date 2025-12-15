#!/usr/bin/env python3
import rospy
from flapper.multi_flapper.application.dual_drone_navigator import NavigatorDual

if __name__ == "__main__":
    # 引数などで自身のIDと相手のIDを指定して起動
    my_id = rospy.get_param("~drone_id", 1)
    rospy.init_node(f"navigator_dual{my_id}")
    other_id = 2 if my_id == 1 else 1
    nav = NavigatorDual(my_id, other_id, theta_scale=0.1)
    rospy.spin()
