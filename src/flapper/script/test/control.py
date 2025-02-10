#!/usr/bin/env python   

import motioncapture
import rospy
from sensor_msgs.msg import Joy
from threading import Thread
import time
from pynput import keyboard
import math

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
 
host_name = '192.168.1.135'
 
mocap_system_type = 'optitrack'

rigid_body_name = 'flapper'
chest_name = 'flapper_chest'
hand_name = 'flapper_hand'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90                                                                                     
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

class Quaternion():
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

# flapper 制御用インターフェース
class CflibApiInterface():
    def __init__(self, uri, orientation_std_dev=8.0e-3):
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.open_link(uri)
        self.wait_until_connected()
        self.orientation_std_dev = orientation_std_dev
        self.send_full_pose = send_full_pose
        self.adjust_orientation_sensitivity()
        self.activate_kalman_estimator()
        self.activate_pid_controller()
        self.tune_pid_gains()

    def wait_until_connected(self):
        try:
            while not self.cf.is_connected():
                time.sleep(0.1)
        except Exception as e:
            print(f"Error during connection: {e}")
            raise
    def adjust_orientation_sensitivity(self):
        # デフォルトの orientation_std_dev は低すぎる可能性があるので適宜調整
        self.cf.param.set_value('locSrv.extQuatStdDev', self.orientation_std_dev)

    def activate_kalman_estimator(self):
        self.cf.param.set_value('stabilizer.estimator', '2')

    def activate_pid_controller(self):
        self.cf.param.set_value('stabilizer.controller', '1')

    def tune_pid_gains(self):
        self.cf.param.set_value('posCtlPid.xKp', 1.0)
        self.cf.param.set_value('posCtlPid.yKp', 1.0)
        self.cf.param.set_value('posCtlPid.xKd', 0.1)
        self.cf.param.set_value('posCtlPid.xKd', 0.1)
        self.cf.param.set_value('pid_attitude.pitch_kp', 26.0)

    def send_pose(self, pos, quat, send_full_pose):
        # """
        # Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
        # This is going to be forwarded to the Crazyflie's position estimator.
        # """
        if send_full_pose:
            self.cf.extpos.send_extpose(pos[0], pos[1], pos[2], quat.x, quat.y, quat.z, quat.w)
        else:
            self.cf.extpos.send_extpos(x, y, z)

    def takeoff(self, absolute_height_m, duration_s):
        self.cf.high_level_commander.takeoff(absolute_height_m, duration_s)
        rospy.loginfo("take off!")
        time.sleep(duration_s + 2)
        rospy.loginfo("stabilized")

    def land(self, absolute_height_m, duration_s):
        self.cf.high_level_commander.land(absolute_height_m, duration_s)
        rospy.loginfo("landing!")
        time.sleep(duration_s + 2)
        rospy.loginfo("landed")

    def stop(self):
        rospy.loginfo("stop")
        self.cf.high_level_commander.stop()

    def go_to(self, x, y, z, duration_s):
        self.cf.high_level_commander.go_to(x, y, z, 0, duration_s)
        time.sleep(0.1)
        
class PoseSender(Thread):
    def __init__(self, drone_control_client, position_getter, send_full_pose = True):
        Thread.__init__(self)
        self.position_getter = position_getter
        self.drone_control_client = drone_control_client
        self.send_full_pose = send_full_pose
        self.start()

    def run(self):
        self.running = True
        while self.running:
            if hasattr(self.position_getter, "body_pos") and hasattr(self.position_getter, "body_rot"):
                self.drone_control_client.send_pose(self.position_getter.body_pos, self.position_getter.body_rot, self.send_full_pose)

    def close(self):
        self.running = False
        print("pose sender closed")

class DroneActionHandler:
    def __init__(self, drone_control_client):
        self.drone_control_client = drone_control_client
        self.flying = False

    def handle_takeoff(self):
        if not self.flying:
            print("taking off")
            self.drone_control_client.takeoff(0.7, 2.0)
            time.sleep(3.0)
            print("taken off")
            self.flying = True

    def handle_land(self, height=0):
        if self.flying:
            print("landing")
            self.drone_control_client.land(height + 0.27, 2.0)
            time.sleep(3.0)
            print("landed")
            self.flying = False

    def hover_above(self, pos):
        if self.flying:
            x, y, z = pos
            self.drone_control_client.go_to(x, y, z, 5)

    def stop(self):
        self.drone_control_client.stop()
        print("motor stopped")

        
if __name__ == '__main__':
    
        cflib.crtp.init_drivers()
        mocap_wrapper = MocapWrapper(rigid_body_name, chest_name, hand_name)     
        cflib_api_interface = CflibApiInterface(uri, 0.06)
        pose_sender = PoseSender(cflib_api_interface, mocap_wrapper)
        drone_action_handler = DroneActionHandler(cflib_api_interface)
        drone_action_setter = DroneActionSetter(drone_action_handler, mocap_wrapper)
        keyboard_listener = KeyboardListener(drone_action_handler, drone_action_setter)
      
