#!/usr/bin/env python3
import rospy
from cf_client import CfClient
from teleop import Teleop
from navigator import Navigator
from cf_logger import CfLogger
from cflib.utils import uri_helper
import traceback

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

def main():
    rospy.init_node('main')
    cf_logger = CfLogger()
    cf_client = CfClient(uri, cf_logger)
    navigator = Navigator(cf_client)
    teleop = Teleop(navigator)
    rospy.spin()

if __name__ == '__main__':
    main()
