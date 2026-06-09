#!/usr/bin/env python   

import motioncapture
import rospy
from sensor_msgs.msg import Joy
from threading import Thread
import time
from pynput import keyboard

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
 
host_name = '192.168.1.135'
 
mocap_system_type = 'optitrack'

rigid_body_name = 'flapper'

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


class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)
        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        print("mocap connecting")
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        print("mocap connected")
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    if self.on_pose:
                        pos = obj.position
                        rot = Quaternion(obj.rotation.x, -obj.rotation.z, obj.rotation.y, obj.rotation.w)
                        self.on_pose([pos[0], -pos[2], pos[1], rot])

# flapper 制御用インターフェース
class CflibApiInterface():
    def __init__(self, cf, orientation_std_dev=8.0e-3, send_full_pose=True):
        self.cf = cf
        self.orientation_std_dev = orientation_std_dev
        self.send_full_pose = send_full_pose
        self.adjust_orientation_sensitivity()
        self.activate_kalman_estimator()
        self.activate_pid_controller()
        self.tune_pid_gains()
        self.set_servo_neutral()

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
        self.cf.param.set_value('pid_attitude.roll_kp', 10.0)
        self.cf.param.set_value('pid_attitude.pitch_kp', 13.0)
        self.cf.param.set_value('pid_attitude.roll_kd', 10.0)
        self.cf.param.set_value('pid_attitude.pitch_kd', 10.0)
        self.cf.param.set_value('pid_attitude.yaw_kp', 120)
        self.cf.param.set_value('pid_rate.yaw_kp', 320)
   
    def set_servo_neutral(self):
        print("neutralized")
        self.cf.param.set_value('flapper.servYawNeutr', 52)
        # self.cf.param.set_value('flapper.servPitchNeutr', 50)

    def send_extpose_quat(self, x, y, z, quat):
        # """
        # Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
        # This is going to be forwarded to the Crazyflie's position estimator.
        # """
        if self.send_full_pose:
            self.cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
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

    def go_to(self, x, y, z):
        self.cf.high_level_commander.go_to(x, y, z, 0, 0.1, True)
        time.sleep(0.1)

class JoyReceiver():
    threshold = 0.05
    
    def __init__(self):
        rospy.init_node('joy_interpreter', anonymous=True)
        rospy.Subscriber('joy', Joy, self.joy_callback)

    def joy_callback(self, data):
        self.axes = list(map(lambda x: self.clamp(x), data.axes[:11]))
        self.buttons = data.buttons[:-1]

    def clamp(self, value):
        return value if abs(value) > self.threshold else 0

class Controller():
    def __init__(self, cf_api_if, input_receiver):
        self.receiver = input_receiver
        self.cf_api_if = cf_api_if
        self.flying = False

    def on_press(self, key):
        if key == keyboard.Key.esc:                                                          
            self.running = False
            self.cf_api_if.stop()
            return False  # Escキーが押されたらリスナーを停止   

    def run(self):
        self.running = True
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()
        while self.running:
            y, x, yaw, l2_push, r2_push, z, gyro1, gyro2, gyro3, horizen, vertical= self.receiver.axes
            square, cross, circle, triangle, l1, r1, l2, r2, share, option, l_stick, r_stick, home = self.receiver.buttons
            if home == 1:
                self.cf_api_if.stop()
                self.flying = False
            elif horizen == -1 and square == 1:
                self.cf_api_if.land(0.3, 2.0)
                self.flying = False
            elif horizen == 1 and circle == 1:
                self.cf_api_if.takeoff(1.0, 2.0)
                self.flying = True
            elif self.flying and (x != 0 or y != 0 or z != 0):
                self.cf_api_if.go_to(x/25, y/25, z/50)
    
        
if __name__ == '__main__':
    
        cflib.crtp.init_drivers()

        mocap_wrapper = MocapWrapper(rigid_body_name)
    
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            try:
                cf = scf.cf
                joy_receiver = JoyReceiver()
                cflib_api_interface = CflibApiInterface(cf, 0.06, True)
                controller = Controller(cflib_api_interface, joy_receiver)
                mocap_wrapper.on_pose = lambda pose: cflib_api_interface.send_extpose_quat(pose[0],
                                                                                           pose[1],
                                                                                           pose[2],
                                                                                           pose[3])
                controller.run()
            except KeyboardInterrupt:
                controller.stop()
                mocap_wrapper.close()
        mocap_wrapper.close()

