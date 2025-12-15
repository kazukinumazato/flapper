#!/usr/bin/env python
import rospy
import time
import math
from geometry_msgs.msg import PoseStamped

class PidController:
    p_gain = 1
    i_gain = 1
    d_gain = 1

    def __init__(goal, curr, rate=10):
        self.dt = 1/rate
        self.goal = goal
        self.curr = curr
        self.err_i = 0
        self.err_pre = 0
        self.vel = 0

    def set_goal(goad):
        self.goal = goal

    def pid(goal, curr):
        err = goal - curr
        self.err_i += err
        p_term = err * p_gain
        i_term = err_i * i_gain
        d_term = (err - self.err_pre) / self.dt * d_gain
        self.vel = p_term + i_term + d_term
        self.err_pre = err
        time.sleep(self.dt)

    def pid_callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        R = quaternion_to_rotation_matrix(data.orientation)
        print(R)
        x, y, z = data.pose
        yaw = math.acos(data.orientation.w) * 2
        local_pos = R @ np.array([[x], [y], [z]])
        self.curr = local_pos.append(yaw) 
        self.err = self.goal - self.curr
        
        

def quaternion_to_rotation_matrix(q):
    """
    クォータニオンを回転行列に変換
    q: [w, x, y, z] のリストまたはタプル
    """
    w, x, y, z = q
    # クォータニオンを正規化
    norm = np.sqrt(w**2 + x**2 + y**2 + z**2)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    # 回転行列
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    return R
    
def flapper_controller():
    rospy.init_node('mocap_listener', anonymous=True)

    rospy.Subscriber("mocap/pose", PoseStamped, pid_callback)

    rospy.spin()

if __name__ == '__main__':
    flapper_controller()
