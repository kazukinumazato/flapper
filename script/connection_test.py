#!/usr/bin/env python   

import rospy
import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper
import logging

from threading import Thread

# flapper 操作指令クラス
class CfClient():
    def __init__(self, uri):
        cflib.crtp.init_drivers()

        self.cf = Crazyflie(rw_cache='./cache')

        self.cf.connected.add_callback(self.connected_callback)
        self.cf.fully_connected.add_callback(self.fully_connected_callback)
        self.cf.disconnected.add_callback(self.disconnected_callback)
        self.cf.connection_failed.add_callback(self.connection_failed_callback)
        self.cf.connection_lost.add_callback(self.connection_lost_callback)
        self.cf.open_link(uri)
        Thread(target=self.keyboard_interrupt_callback).start()


    def keyboard_interrupt_callback(self):
        while True:
            if rospy.is_shutdown():
                self.cf.close_link()
                break
        
    def connected_callback(self, uri):
        rospy.loginfo('Connected to %s' % uri)
    
    def fully_connected_callback(self, uri):
        rospy.loginfo('Fully connected to %s' % uri)
        
    def connection_failed_callback(self, uri, msg):
        rospy.logerr('Connection to %s failed: %s' % (uri, msg))
                                                                
    def connection_lost_callback(self, uri, msg):
        rospy.logerr('Connection to %s lost: %s' % (uri, msg))
        
    def disconnected_callback(self, uri):
        rospy.logerr('Disconnected from %s' % uri)

if __name__ == '__main__':
    rospy.init_node('test')
    uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    CfClient(uri)
