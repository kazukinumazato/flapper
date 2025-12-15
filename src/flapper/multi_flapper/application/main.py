#!/usr/bin/env python3
import rospy
from flapper.multi_flapper.service.cf_client import CfClient
from flapper.multi_flapper.service.command_router import CommandRouter
from flapper.multi_flapper.service.cf_logger import CfLogger
from cflib.utils import uri_helper
from flapper.multi_flapper.domain.motion_manager import MotionManager
import traceback

uri1 = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")
uri2 = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E8")


def main():
    rospy.init_node("multi_drone_manager")

    # 1機目の設定
    logger1 = CfLogger(drone_id=1)
    cf1 = CfClient(uri1, logger1, drone_id=1)
    manager1 = MotionManager(cf1, drone_id=1, hand_id=1)

    # 2機目の設定
    logger2 = CfLogger(drone_id=2)
    cf2 = CfClient(uri2, logger2, drone_id=2)
    manager2 = MotionManager(cf2, drone_id=2, hand_id=2)

    router = CommandRouter(manager1, manager2)
    rospy.spin()


if __name__ == "__main__":
    main()
