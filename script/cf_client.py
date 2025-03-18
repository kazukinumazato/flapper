#!/usr/bin/env python   

import rospy
import cflib
from cflib.crazyflie import Crazyflie
from geometry_msgs.msg import PoseStamped

from threading import Thread

# flapper 操作指令クラス
class CfClient():
    def __init__(self, uri, cf_logger):
        self.cf_logger = cf_logger
        cflib.crtp.init_drivers()

        self.cf = Crazyflie(rw_cache='./cache')

        self.cf.connected.add_callback(self.connected_callback)
        self.cf.fully_connected.add_callback(self.fully_connected_callback)
        self.cf.disconnected.add_callback(self.disconnected_callback)
        self.cf.connection_failed.add_callback(self.connection_failed_callback)
        self.cf.connection_lost.add_callback(self.connection_lost_callback)

        self.cf.open_link(uri)

    def keyboard_interrupt_callback(self):
        while True:
            rospy.sleep(0.1)
            if rospy.is_shutdown():
                self.stop()
                self.close()
                break
    
    def connected_callback(self, uri):
        rospy.loginfo('Connected to %s' % uri)
        
        Thread(target=self.keyboard_interrupt_callback).start()

        for log_config in self.cf_logger.log_configs:
            try:
                self.cf.log.add_config(log_config)
            except AttributeError as e:
                rospy.logerr('Could not add log config, bad configuration.')
                                                                                                
        for log_config in self.cf_logger.log_configs:
            try:
                log_config.start()
            except KeyError as e:
                rospy.logerr('Could not start log configuration,'
                             '{} not found in TOC'.format(str(e)))
    
    def fully_connected_callback(self, uri):
        rospy.loginfo('Fully connected to %s' % uri)
        self.adjust_orientation_sensitivity()
        self.activate_kalman_estimator()
        self.activate_pid_controller()
        self.tune_pid_gains()
        self.mocap_sub = rospy.Subscriber('mocap_node/mocap/flapper/pose',
                                          PoseStamped,
                                          self.mocap_sub_callback)
        
    def connection_failed_callback(self, uri, msg):
        rospy.logerr('Connection to %s failed: %s' % (uri, msg))
                                                                
    def connection_lost_callback(self, uri, msg):
        rospy.logerr('Connection to %s lost: %s' % (uri, msg))
        
    def disconnected_callback(self, uri):
        rospy.logerr('Disconnected from %s' % uri)

    def _param_callback(self, name, value):
        """Generic callback registered for all the groups"""
        print('{0}: {1}'.format(name, value))
        
    def adjust_orientation_sensitivity(self, orientation_std_dev=0.06):
        self.cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)

    def activate_kalman_estimator(self):
        self.cf.param.set_value('stabilizer.estimator', '2')

    def activate_pid_controller(self):
        self.cf.param.set_value('stabilizer.controller', '1')

    def tune_pid_gains(self):
        self.cf.param.set_value('posCtlPid.xKp', 1.0)
        self.cf.param.set_value('posCtlPid.yKp', 1.0)
        self.cf.param.set_value('posCtlPid.xKd', 0.1)
        self.cf.param.set_value('posCtlPid.yKd', 0.1)
        self.cf.param.set_value('pid_attitude.pitch_kp', 52.0)

    def mocap_sub_callback(self, msg):
        self.send_pose(msg.pose.position, msg.pose.orientation)
        
    def send_pose(self, pos, quat, send_full_pose = True):
        if send_full_pose:
            self.cf.extpos.send_extpose(pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w)
        else:
            self.cf.extpos.send_extpos(pos.x, pos.y, pos.z)
        
    def takeoff(self, height = 1, duration = 3):
        self.cf.high_level_commander.takeoff(height, duration)
        
    def land(self, height = 0, duration = 3, yaw = 0.0):
        group_mask = 0
        self.cf.high_level_commander.land(height, duration, group_mask, yaw)
        
    def stop(self):
        self.cf.high_level_commander.stop()
        
    def go_to(self, x, y, z, yaw ,duration_s, relative=False):
        self.cf.high_level_commander.go_to(x, y, z, yaw, duration_s, relative)

    def close(self):
        self.cf.close_link()
        
