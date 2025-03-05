#!/usr/bin/env python   

import rospy
from cflib.crazyflie.log import LogConfig
from flapper.msg import Rpm, Stabilizer
from geometry_msgs.msg import Vector3, Quaternion

class LogData():
    def __init__(self, name, pub_class, attribute_name, topic_name, data_list):
        self.name = name
        self.pub_class = pub_class
        self.attribute_name = attribute_name
        self.topic_name = topic_name
        self.data_list = data_list

class CfLogger():
    def __init__(self):
        self.log_list = [LogData('estimate_position', Vector3, 'log_estimate_position_pub', 'log/state_estimate/position',
                                 [['stateEstimate.x', 'float', 'x'],
                                  ['stateEstimate.y', 'float', 'y'],
                                  ['stateEstimate.z', 'float', 'z'],
                                 ]),
                         LogData('estimate_orientation', Quaternion, 'log_estimate_orientation_pub', 'log/state_estimate/orientation',
                                 [['stateEstimate.qw', 'float', 'w'],
                                  ['stateEstimate.qx', 'float', 'x'],
                                  ['stateEstimate.qy', 'float', 'y'],
                                  ['stateEstimate.qz', 'float', 'z'],
                                 ]),
                         LogData('estimate_acc', Vector3, 'log_estimate_acc_pub', 'log/state_estimate/acc',
                                 [['stateEstimate.ax', 'float', 'x'],
                                  ['stateEstimate.ay', 'float', 'y'],
                                  ['stateEstimate.az', 'float', 'z'],
                                 ]),
                         LogData('estimate_vel', Vector3, 'log_estimate_vel_pub', 'log/state_estimate/vel',
                                 [['stateEstimate.vx', 'float', 'x'],
                                  ['stateEstimate.vy', 'float', 'y'],
                                  ['stateEstimate.vz', 'float', 'z']
                                 ]),
                         LogData('Stabilizer', Stabilizer, 'log_stabilizer_pub', 'log/stabilizer',
                                 [['stabilizer.roll', 'float', 'roll'],
                                  ['stabilizer.pitch', 'float', 'pitch'],
                                  ['stabilizer.yaw', 'float', 'yaw'],
                                  ['stabilizer.thrust', 'float', 'thrust']
                                 ]),
                         LogData('rpm', Rpm, 'log_rpm_pub', 'log/rpm',
                                 [['rpm.m1', 'uint16_t', 'm1'],
                                  ['rpm.m2', 'uint16_t', 'm2'],
                                  ['rpm.m3', 'uint16_t', 'm3'],
                                  ['rpm.m4', 'uint16_t', 'm4']
                                 ]),
                         LogData('gyro_omega', Vector3, 'log_gyro_omega_pub', 'log/gyro/omega',
                                 [['gyro.x', 'float', 'x'],
                                  ['gyro.y', 'float', 'y'],
                                  ['gyro.z', 'float', 'z']
                                 ]),
                         LogData('gyro_raw', Vector3, 'log_gyro_raw_pub', 'log/gyro/raw',
                                 [['gyro.xRaw', 'int16_t', 'x'],
                                  ['gyro.yRaw', 'int16_t', 'y'],
                                  ['gyro.zRaw', 'int16_t', 'z']]),
                         LogData('gyro_variance', Vector3, 'log_gyro_variance_pub', 'log/gyro/variance',
                                 [['gyro.xVariance', 'float', 'x'],
                                  ['gyro.yVariance', 'float', 'y'],
                                  ['gyro.zVariance', 'float', 'z']
                                 ]),
                         LogData('acc', Vector3, 'acc_pub', 'log/acc',
                                 [['acc.x', 'float', 'x'],
                                  ['acc.y', 'float', 'y'],
                                  ['acc.z', 'float', 'z']
                                 ]
                         )
        ]

        for log_data in self.log_list:
            setattr(self, log_data.attribute_name, rospy.Publisher(log_data.topic_name, log_data.pub_class))

        self.log_configs = []
        self.generate_log_configs()

    def generate_log_configs(self):
        # The definition of the logconfig can be made before connecting
        for idx ,log_data in enumerate(self.log_list):
            log_config = LogConfig(name=f'Log{idx}', period_in_ms=50)
            for data in log_data.data_list:
                log_config.add_variable(data[0], data[1])
            log_config.data_received_cb.add_callback(self.generate_log_data_callback(log_data))
            log_config.error_cb.add_callback(self.log_error_callback)
            self.log_configs.append(log_config)
    def log_error_callback(self, logconf, msg):
        rospy.logerr('Error when logging %s: %s' % (logconf.name, msg))

    def generate_log_data_callback(self, log_data):
        def log_data_callback(timestamp, data, logconf):
            self.log_data_callback_with_name(timestamp, data, logconf, log_data)
        return log_data_callback

    def log_data_callback_with_name(self, timestamp, data, logconf, log_data):
        pub_instance = log_data.pub_class()
        for data_info in log_data.data_list:
            for name, value in data.items():
                if name == data_info[0]:
                    members = data_info[2:]
                    self.set_nested_attr(pub_instance, members, value)
        pub = getattr(self, log_data.attribute_name)
        pub.publish(pub_instance)

    def set_nested_attr(self, obj, attrs, value):
            current = obj
            for idx, attr in enumerate(attrs):
                if len(attrs) - 1 == idx:
                    setattr(current, attr, value)
                current = getattr(current, attr)
          
