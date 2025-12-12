#!/usr/bin/env python3
import rospy

# from cf_client2 import CfClient                 # <- オリジナルをコメントアウト
from script.multi_flapper.sim_mock_cf_client import (
    MockCfClient as CfClient,
)  # <- モッククライアントをインポートして置き換え
from script.multi_flapper.teleop_dual import Teleop
from script.multi_flapper.cf_logger_unique import CfLogger
from cflib.utils import uri_helper
from script.multi_flapper.motion_manager_unique import MotionManager
import traceback
from script.multi_flapper.navigator_dual import NavigatorDual

uri1 = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")
uri2 = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E8")


def main():
    rospy.init_node("multi_drone_manager")

    # 1機目の設定
    logger1 = CfLogger(drone_id=1)
    cf1 = CfClient(uri1, logger1, drone_id=1)
    manager1 = MotionManager(cf1, drone_id=1, hand_id=1)

    navigator1 = NavigatorDual(my_id=1, other_id=2)

    # 2機目の設定
    logger2 = CfLogger(drone_id=2)
    cf2 = CfClient(uri2, logger2, drone_id=2)
    manager2 = MotionManager(cf2, drone_id=2, hand_id=2)

    navigator2 = NavigatorDual(my_id=2, other_id=1)

    teleop = Teleop(manager1, manager2)
    rospy.spin()


if __name__ == "__main__":
    main()
