#!/usr/bin/env python3
import rospy
from flapper.single_flapper.application.navigation.navigator import Navigator

if __name__ == "__main__":
    rospy.init_node("navigator")
    Navigator()
    rospy.spin()
