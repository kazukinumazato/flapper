

#!/usr/bin/env python   

import rospy
import cflib
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper
from geometry_msgs.msg import Vector3, Quaternion

from threading import Thread
from flapper.msg import Rpm, Stabilizer
import traceback

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

class LogData():
    def __init__(self, variable_name, pub_class, attribute_name, topic_name, data_list):
        self.variable_name = variable_name
        self.pub_class = pub_class
        self.attribute_name = attribute_name
        self.topic_name = topic_name
        self.data_list = data_list

# flapper 操作指令クラス
class CfClient():
    def __init__(self, uri):
        cflib.crtp.init_drivers()

        self.cf = Crazyflie(rw_cache='./cache')

        self.cf.connected.add_callback(self.connected_callback)
        self.cf.disconnected.add_callback(self.disconnected_callback)
        self.cf.connection_failed.add_callback(self.connection_failed_callback)
        self.cf.connection_lost.add_callback(self.connection_lost_callback)

        self.log_list = [LogData('estimate_position', Vector3, 'log_estimate_position_pub', 'log/state_estimate/position',
                                 [['stateEstimate.x', 'float', 'x'],
                                  ['stateEstimate.y', 'float', 'y'],
                                  ['stateEstimate.z', 'float', 'z'],
                                 ]),
                         # LogData('estimate_orientation', Quaternion, 'log_estimate_orientation_pub', 'log/state_estimate/orientation',
                         #         [['stateEstimate.qw', 'float', 'w'],
                         #          ['stateEstimate.qx', 'float', 'x'],
                         #          ['stateEstimate.qy', 'float', 'y'],
                         #          ['stateEstimate.qz', 'float', 'z'],
                         #         ]),
                         # LogData('estimate_acc', Vector3, 'log_estimate_acc_pub', 'log/state_estimate/acc',
                         #         [['stateEstimate.ax', 'float', 'x'],
                         #          ['stateEstimate.ay', 'float', 'y'],
                         #          ['stateEstimate.az', 'float', 'z'],
                         #         ]),
                         # LogData('estimate_vel', Vector3, 'log_estimate_vel_pub', 'log/state_estimate/vel',
                         #         [['stateEstimate.vx', 'float', 'x'],
                         #          ['stateEstimate.vy', 'float', 'y'],
                         #          ['stateEstimate.vz', 'float', 'z']
                         #         ]),
                         # LogData('Stabilizer', Stabilizer, 'log_stabilizer_pub', 'log/stabilizer',
                         #         [['stabilizer.roll', 'float', 'roll'],
                         #          ['stabilizer.pitch', 'float', 'pitch'],
                         #          ['stabilizer.yaw', 'float', 'yaw'],
                         #          ['stabilizer.thrust', 'float', 'thrust']
                         #         ]),
                         # LogData('rpm', Rpm, 'log_rpm_pub', 'log/rpm',
                         #         [['rpm.m1', 'uint16_t', 'm1'],
                         #          ['rpm.m2', 'uint16_t', 'm2'],
                         #          ['rpm.m3', 'uint16_t', 'm3'],
                         #          ['rpm.m4', 'uint16_t', 'm4']
                         #         ]),
                         # LogData('gyro_omega', Vector3, 'log_gyro_omega_pub', 'log/gyro/omega',
                         #         [['gyro.x', 'float', 'x'],
                         #          ['gyro.y', 'float', 'y'],
                         #          ['gyro.z', 'float', 'z']
                         #         ]),
                         # LogData('gyro_raw', Vector3, 'log_gyro_raw_pub', 'log/gyro/raw',
                         #         [['gyro.xRaw', 'int16_t', 'x'],
                         #          ['gyro.yRaw', 'int16_t', 'y'],
                         #          ['gyro.zRaw', 'int16_t', 'z']]),
                         # LogData('gyro_variance', Vector3, 'log_gyro_variance_pub', 'log/gyro/variance',
                         #         [['gyro.xVariance', 'float', 'x'],
                         #          ['gyro.yVariance', 'float', 'y'],
                         #          ['gyro.zVariance', 'float', 'z']
                         #         ]),
                         
        ]

        for log_data in self.log_list:
            rospy.loginfo(f'set {log_data.attribute_name}')
            setattr(self, log_data.attribute_name, rospy.Publisher(log_data.topic_name, log_data.pub_class))

        
        self.cf.open_link(uri)
        if not self.cf.is_connected():
            return
        self.adjust_orientation_sensitivity()
        self.activate_kalman_estimator()
        self.activate_pid_controller()
        self.tune_pid_gains()


    def connected_callback(self, uri):
        rospy.loginfo('Connected to %s' % uri)

        
        Thread(target=self.keyboard_interrupt_callback).start()

        # The definition of the logconfig can be made before connecting
        for idx ,log_data in enumerate(self.log_list):
            setattr(self, f'log_config_{idx}', LogConfig(name=f'Log{idx}', period_in_ms=100))
            for data in log_data.data_list:
                rospy.loginfo(data[0])
                self.log_config.add_variable(data[0], data[1])
            try:
                log_config = getattr(self, f'log_config_{idx}')
                self.cf.log.add_config(log_config)
                log_config.data_received_cb.add_callback(self.log_data_callback)
                log_config.error_cb.add_callback(self.log_error_callback)
                log_config.start()
            except KeyError as e:
                rospy.logerr('Could not start log configuration,'
                             '{} not found in TOC'.format(str(e)))
            except AttributeError as e:
                rospy.logerr('Could not add log config, bad configuration.')
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())

    def keyboard_interrupt_callback(self):
        while True:
            if rospy.is_shutdown():
                self.stop()
                self.close()
                break

    def log_error_callback(self, logconf, msg):
        rospy.logerr('Error when logging %s: %s' % (logconf.name, msg))


    def log_data_callback(self, timestamp, data, logconf):
        def set_nested_attr(obj, attrs, value):
            current = obj
            for attr in attrs[:-1]:
                if not hasattr(current, attr):
                    setattr(current, attrs[-1], value)
                current = getattr(current, attr)
          
        data_items = data.items()
        for log_data in self.log_list:
            pub_instance = log_data.pub_class()
            for data_info in log_data.data_list:
                for item in data_items:
                    name, value = item
                    print(name)
                    print(value)
                    if name == data_info[0]:
                        members = data_info[2:]
                        set_nested_attr(pub_instance, members, value)
                        break
            pub = getattr(self, log_data.attribute_name)
            pub.publish(pub_instance)
        
    def connection_failed_callback(self, uri, msg):
        rospy.logerr('Connection to %s failed: %s' % (uri, msg))
                                                                
    def connection_lost_callback(self, uri, msg):
        rospy.logerr('Connection to %s lost: %s' % (uri, msg))
        
    def disconnected_callback(self, uri):
        rospy.logerr('Disconnected from %s' % uri)  
        
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
        self.cf.param.set_value('posCtlPid.xKd', 0.1)
        self.cf.param.set_value('pid_attitude.pitch_kp', 26.0)
        
    def send_pose(self, pos, quat, send_full_pose = True):
        if send_full_pose:
            self.cf.extpos.send_extpose(pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w)
        else:
            self.cf.extpos.send_extpos(pos.x, pos.y, pos.z)
        
    def takeoff(self, height, duration):
        self.cf.high_level_commander.takeoff(height, duration)
        
    def land(self, height, duration):
        self.cf.high_level_commander.land(height, duration)
        
    def stop(self):
        self.cf.high_level_commander.stop()
        
    def go_to(self, x, y, z, yaw ,duration_s):
        self.cf.high_level_commander.go_to(x, y, z, yaw, duration_s)

    def close(self):
        self.cf.close_link()
        
