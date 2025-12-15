#!usr/bin/env python
import rospy
from std_msgs.msg import Empty, Int64
from geometry_msgs.msg import Pose
from flapper.shared.variables import *

"""
ROSトピックにコマンドを送る
"""


class FlapperCommander:
    def __init__(self):
        self.state_sub = rospy.Subscriber("/state", Int64, self.state_callback)
        self.takeoff_pub = rospy.Publisher("/teleop_command/takeoff", Empty)
        self.land_pub = rospy.Publisher("/teleop_command/land", Empty)
        self.halt_pub = rospy.Publisher("/teleop_command/halt", Empty)
        self.palm_land_pub = rospy.Publisher("palm_land", Pose)

        self.state = RobotState.START

    def state_callback(self, msg):
        self.state = msg.data

    def takeoff(self):
        self.takeoff_pub.publish()

    def land(self):
        self.land_pub.publish()

    def palm_land(self, yaw):
        self.palm_land_pub.publish(yaw)

    def stop(self):
        self.halt_pub.publish()
