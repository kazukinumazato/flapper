import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped
from script.shared.variables import RobotState

"""
各種動きを定義
"""


class MotionManager:
    def __init__(self, drone_client):
        self.drone_client = drone_client
        self.state_pub = rospy.Publisher("state", Int64, queue_size=1)
        self.drone_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/flapper/pose", PoseStamped, self.drone_pose_sub_callback
        )
        self.hand_pose_sub = rospy.Subscriber(
            "mocap_node/mocap/hand/pose", PoseStamped, self.hand_pose_sub_callback
        )
        self.robot_state = RobotState.START

    def drone_pose_sub_callback(self, msg):
        self.drone_pose = msg.pose

    def hand_pose_sub_callback(self, msg):
        self.hand_pose = msg.pose

    def takeoff(self, height=1, duration=3, threshold=0.01, timeout=10):
        if self.robot_state == RobotState.HOVER:
            rospy.loginfo("The robot is already hovering.")
            return
        if self.robot_state != RobotState.START and self.robot_state != RobotState.STOP:
            rospy.logwarn("The robot is not in the start state. Unable to take off.")
            return
        if not hasattr(self, "drone_pose"):
            rospy.logwarn("drone pose is not acquired yet. Unable to take off.")
            return
        rospy.loginfo("takeoff started")
        self.drone_client.takeoff(height, duration)
        self.robot_state = RobotState.TAKEOFF
        self.state_pub.publish(RobotState.TAKEOFF)
        rospy.sleep(5)
        self.robot_state = RobotState.HOVER
        self.state_pub.publish(RobotState.HOVER)
        # start_t = rospy.get_time()
        # while rospy.get_time() <= start_t + timeout:
        #     rospy.sleep(0.1)
        #     if abs(self.drone_pose.position.z - height) <= threshold:
        #         rospy.sleep(2)
        #         rospy.loginfo('takeoff succeeded')
        #         self.robot_state = RobotState.HOVER
        #         self.state_pub.publish(RobotState.HOVER)
        #         break
        #     if rospy.is_shutdown():
        #         print('rospy shutdown')
        #         break

    def land(self, duration=3):
        if self.robot_state == RobotState.START or self.robot_state == RobotState.STOP:
            rospy.loginfo("The robot is already landed.")
            return
        if self.robot_state == RobotState.LAND:
            rospy.loginfo("The robot is already landing.")
            return
        rospy.loginfo("land started")
        self.drone_client.land(0, duration)
        self.robot_state = RobotState.LAND
        self.state_pub.publish(RobotState.LAND)
        rospy.sleep(duration)
        rospy.loginfo("land succeeded")
        self.robot_state = RobotState.STOP
        self.state_pub.publish(RobotState.STOP)

    def approach(self, x, y, z, yaw=0, duration_s=0.1, relative=False):
        if (
            not self.robot_state == RobotState.HOVER
            and not self.robot_state == RobotState.APPROACH
        ):
            rospy.logwarn("The robot is not ready to approach. Unable to approach.")
            return
        rospy.loginfo("approaching target position: x: %f, y: %f, z: %f" % (x, y, z))
        self.drone_client.go_to(x, y, z, 0, duration_s, relative)
        self.robot_state = RobotState.APPROACH
        self.state_pub.publish(RobotState.APPROACH)

    def stop(self):
        self.drone_client.stop()
        rospy.loginfo("drone stopped")
        self.robot_state = RobotState.STOP
        self.state_pub.publish(RobotState.STOP)

    def palm_land(self, yaw, duration=3):
        if self.robot_state == RobotState.START or self.robot_state == RobotState.STOP:
            rospy.loginfo("The robot is already landed.")
            return
        rospy.loginfo("palm land started")
        self.robot_state = RobotState.PALM_LAND
        self.state_pub.publish(RobotState.PALM_LAND)
        hand_height = self.hand_pose.position.z
        self.drone_client.land(hand_height, duration, yaw)
        rospy.sleep(duration)
        rospy.loginfo("palm land succeeded")
        self.robot_state = RobotState.STOP
        self.state_pub.publish(RobotState.STOP)
