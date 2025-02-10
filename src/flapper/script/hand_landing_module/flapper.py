#!usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Int64
from variables import *

class Flapper:
    def __init__(self):
        self.state_sub = rospy.Subscriber('/state', Int64,
                                           self.state_callback)
        self.takeoff_pub = rospy.Publisher('/teleop_command/takeoff', Empty)
        self.land_pub = rospy.Publisher('/teleop_command/land', Empty)
        self.approach_pub = rospy.Publisher('/teleop_command/approach', Empty)
        self.palm_land_pub = rospy.Publisher('/teleop_command/palm_land', Empty)
        self.halt_pub = rospy.Publisher('/teleop_command/halt', Empty)
        self.state = RobotState.START

    def state_callback(self, msg):
        self.state = msg.data

    def takeoff(self):
        self.takeoff_pub.publish()

    def land(self):
        self.land_pub.publish()

    def approach(self):
        self.approach_pub.publish()

    def palm_land(self):
        self.palm_land_pub.publish()

    def stop(self):
        self.halt_pub.publish()
