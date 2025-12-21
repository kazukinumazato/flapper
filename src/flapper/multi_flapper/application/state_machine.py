#!/usr/bin/env python3

import rospy
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
import math

# robot.pyからMultiFlapperをインポート
from flapper.multi_flapper.domain.commander import MultiFlapperCommander
from flapper.shared.variables import *
from flapper.shared.util import dist
import traceback


class Start(State):
    def __init__(self):
        State.__init__(self, outcomes=["success", "failure"])
        self.should_start = False
        self.start_sub = rospy.Subscriber("/task_start", Empty, self.start_sub_callback)

    def start_sub_callback(self, _):
        self.should_start = True

    def execute(self, userdata):
        while not self.should_start:
            rospy.sleep(0.1)
            rospy.logdebug_throttle(1.0, "waiting to start")
            if rospy.is_shutdown():
                print("shutdown ros")
                return "failure"
        return "success"


class SingleCommandState(State):
    # MultiFlapperを使うため、このクラスは不要になることが多いが、ここではTakeoff/Landのベースとして残す
    def __init__(self, robot, func, start_flight_state, goal_flight_state, timeout=60):
        State.__init__(self, outcomes=["success", "failure"])
        self.robot = robot
        self.func = func
        self.start_flight_state = start_flight_state
        self.goal_flight_state = goal_flight_state
        self.timeout = timeout

    def execute(self, userdata):
        # 状態チェックはTakeoffとLandのオーバーライドで実行
        pass  # Base class execute is not used


class Takeoff(SingleCommandState):
    # 離陸
    def __init__(self, robot, func):
        # MultiFlapperを渡す
        SingleCommandState.__init__(
            self, robot, func, RobotState.START, RobotState.HOVER
        )

    def execute(self, userdata):
        self.robot.takeoff()  # MultiFlapper経由で一斉離陸

        # 両方が HOVER になるまで待機 (同期)
        while not self.robot.are_both_in_state(RobotState.HOVER):
            if rospy.is_shutdown():
                return "failure"
            rospy.sleep(0.1)

        rospy.loginfo("Both robots have taken off and are now hovering.")
        return "success"


class Land(SingleCommandState):
    # Landステートも同様に同期待機ロジックを実装すべきだが、ここでは割愛し、Takeoff/PalmLandに集中
    def __init__(self, robot, func):
        SingleCommandState.__init__(
            self, robot, func, RobotState.HOVER, RobotState.STOP
        )

    def execute(self, userdata):
        self.robot.land()  # MultiFlapper経由で一斉着陸

        # 両方が STOP になるまで待機
        while not self.robot.are_both_in_state(RobotState.STOP):
            if rospy.is_shutdown():
                return "failure"
            rospy.sleep(0.1)

        rospy.loginfo("Both robots have landed.")
        return "success"


class Approach(State):
    def __init__(self, robot, timeout=120, safety_radius=0.30, threshold=0.1):
        State.__init__(self, outcomes=["stay", "palm_land", "failure"])
        self.robot = robot
        self.approach_start_pub = rospy.Publisher("approach_start", Empty, queue_size=1)
        self.approach_stop_pub = rospy.Publisher("approach_stop", Empty, queue_size=1)

        # 状態遷移判定のため、CF1とHand1を購読する (簡略化)
        self.chest_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/chest/pose", PoseStamped, self.chest_pose_sub_callback
        )
        self.hand1_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/hand1/pose", PoseStamped, self.hand1_pose_sub_callback
        )
        self.drone1_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/flapper1/pose", PoseStamped, self.drone1_pose_sub_callback
        )
        self.hand2_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/hand2/pose", PoseStamped, self.hand2_pose_sub_callback
        )
        self.drone2_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/flapper2/pose", PoseStamped, self.drone2_pose_sub_callback
        )
        self.timeout = timeout
        self.safety_radius = safety_radius
        self.threshold = threshold

    def chest_pose_sub_callback(self, msg):
        self.chest_pose = msg.pose

    def hand1_pose_sub_callback(self, msg):
        self.hand1_pose = msg.pose

    def drone1_pose_sub_callback(self, msg):
        self.drone1_pose = msg.pose
    
    def hand2_pose_sub_callback(self, msg):
        self.hand2_pose = msg.pose

    def drone2_pose_sub_callback(self, msg):
        self.drone2_pose = msg.pose

    def execute(self, userdata):
        self.approach_start_pub.publish(Empty())  # Navigatorに一斉開始を指示
        start_t = rospy.get_time()
        while rospy.get_time() < start_t + self.timeout:
            if rospy.is_shutdown():
                return "failure"
            # MoCapデータが揃っているかチェック
            if (
                not hasattr(self, "chest_pose")
                or not hasattr(self, "hand1_pose")
                or not hasattr(self, "drone1_pose")
                or not hasattr(self, "hand2_pose")
                or not hasattr(self, "drone2_pose")
            ):
                rospy.logwarn(
                    "MoCap data not yet available for Approach transition check."
                )
                rospy.sleep(0.1)
                continue

            chest_pos = self.chest_pose.position
            hand1_pos = self.hand1_pose.position
            drone1_pos = self.drone1_pose.position
            hand2_pos = self.hand2_pose.position
            drone2_pos = self.drone2_pose.position
            distance_hand1_drone1_on_XY_plane = dist(hand1_pos, drone1_pos, True)
            distance_hand1_drone1_z=drone1_pos.z-hand1_pos.z
            distance_chest_hand1 = dist(chest_pos, hand1_pos)
            distance_hand2_drone2_on_XY_plane = dist(hand2_pos, drone2_pos, True)
            distance_hand2_drone2_z=drone2_pos.z-hand2_pos.z
            distance_chest_hand2 = dist(chest_pos, hand2_pos)

            rospy.loginfo(f"distance_chest_hand: 1={distance_chest_hand1}, 2={distance_chest_hand2}")
            if distance_chest_hand1 < self.safety_radius or distance_chest_hand2 < self.safety_radius:
                self.approach_stop_pub.publish(Empty())
                return "stay"
            if distance_hand1_drone1_on_XY_plane < self.threshold and distance_hand2_drone2_on_XY_plane< self.threshold and distance_hand2_drone2_z<35.0 and distance_hand1_drone1_z<35.0:
                self.approach_stop_pub.publish(Empty())
                return "palm_land"
            rospy.sleep(0.1)
        self.approach_stop_pub.publish(Empty())
        return "failure"


class Stay(State):
    def __init__(self, timeout=120, safety_radius=0.30):
        State.__init__(self, outcomes=["approach", "failure"])
        # 状態遷移判定のため、CF1とHand1を購読する (簡略化)
        self.chest_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/chest/pose", PoseStamped, self.chest_pose_sub_callback
        )
        self.hand_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/hand1/pose", PoseStamped, self.hand_pose_sub_callback
        )
        self.timeout = timeout
        self.safety_radius = safety_radius

    def chest_pose_sub_callback(self, msg):
        self.chest_pose = msg.pose

    def hand_pose_sub_callback(self, msg):
        self.hand_pose = msg.pose

    def execute(self, userdata):
        start_t = rospy.get_time()
        while rospy.get_time() < start_t + self.timeout:
            rospy.sleep(0.1)
            if rospy.is_shutdown():
                return "failure"
            if not hasattr(self, "chest_pose") or not hasattr(self, "hand_pose"):
                rospy.logwarn(
                    "Unable to get chest_pose/hand_pose. Check mocap settings."
                )
                continue
            chest_pos = self.chest_pose.position
            hand_pos = self.hand_pose.position
            distance_chest_hand = dist(chest_pos, hand_pos)
            rospy.loginfo(f"distance_chest_hand: {distance_chest_hand}")
            if distance_chest_hand > self.safety_radius:
                return "approach"
        return "failure"


class PalmLand(State):
    def __init__(self, robot):
        State.__init__(self, outcomes=["success", "failure"])
        self.robot = robot

    def execute(self, userdata):
        self.robot.palm_land(yaw=0.0)  # MultiFlapper経由で一斉Palm Land指令

        # 両機が STOP 状態になるまで待機 (同期)
        while not self.robot.are_both_in_state(RobotState.STOP):
            if rospy.is_shutdown():
                return "failure"
            rospy.sleep(0.1)

        rospy.loginfo("Both drones have successfully landed.")
        return "success"


def main():
    rospy.init_node("state_machine")

    # Create a SMACH state machine
    sm = StateMachine(outcomes=["success", "failure"])
    flapper = MultiFlapperCommander()

    # Open the container
    with sm:
        # Add states to the container
        StateMachine.add(
            "Start", Start(), transitions={"success": "Takeoff", "failure": "failure"}
        )
        # Takeoff/Land/PalmLand は MultiFlapper を使って同期
        StateMachine.add(
            "Takeoff",
            Takeoff(flapper, flapper.takeoff),
            transitions={"success": "Stay", "failure": "failure"},
        )
        # Approach/Stayは状態遷移のトリガーのため、ここではCF1のデータのみをチェックする (簡略化)
        StateMachine.add(
            "Approach",
            Approach(flapper),
            transitions={"palm_land": "PalmLand", "stay": "Stay", "failure": "failure"},
        )
        StateMachine.add(
            "Stay", Stay(), transitions={"approach": "Approach", "failure": "failure"}
        )
        StateMachine.add(
            "PalmLand",
            PalmLand(flapper),
            transitions={"success": "success", "failure": "failure"},
        )

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer("smach_server", sm, "/SM_ROOT")
    sis.start()
    outcome = sm.execute()
    if outcome == "failure":
        flapper.land()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(traceback.format_exc())
