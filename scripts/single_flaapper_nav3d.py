#!/usr/bin/env python3
import rospy
from src.single_flapper.application.navigation.navigator_3d import Navigator3D

if __name__ == "__main__":
    try:
        rospy.init_node("navigator")
        nav = Navigator3D(theta_scale=0.1)
        rospy.loginfo("Navigator Node Ready.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
