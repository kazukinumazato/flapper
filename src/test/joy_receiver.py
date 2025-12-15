#!/usr/bin/env python

import rospy                                                                                        
from sensor_msgs.msg import Joy


class JoyReceiver():
    def __init__(self):
        rospy.init_node('joy_receiver', anonymous=True)
        rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.spin()

    def joy_callback(self, data):
        rospy.loginfo(data)

if __name__ == '__main__':
    JoyReceiver()
