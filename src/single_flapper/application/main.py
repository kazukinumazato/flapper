#!/usr/bin/env python3
import rospy
from script.single_flapper.service.cf_client import CfClient
from script.single_flapper.service.command_router import CommandRouter
from script.single_flapper.service.cf_logger import CfLogger
from cflib.utils import uri_helper
from script.single_flapper.domain.motion_manager import MotionManager
import traceback

uri = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E8")


def main():
    rospy.init_node("main")
    cf_logger = CfLogger()
    cf_client = CfClient(uri, cf_logger)
    motion_manager = MotionManager(cf_client)
    router = CommandRouter(motion_manager)
    rospy.spin()


if __name__ == "__main__":
    main()
