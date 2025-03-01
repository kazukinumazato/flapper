#!/usr/bin/env python3

import rospy
from smach import State, StateMachine
from std_msgs.msg import Empty 
import math
from robot import Flapper
from variables import *
import traceback

class Start(State):
        def __init__(self):
                State.__init__(self, outcomes = ['success', 'failure'])
                self.should_start = False
                self.start_sub = rospy.Subscriber('/task_start', Empty, self.start_sub_callback)

        def start_sub_callback(self, _):
                self.should_start = True
                
        def execute(self, userdata):
                while not self.should_start:
                        rospy.sleep(0.1)
                        rospy.logdebug_throttle(1.0, "waiting to start")
                        if rospy.is_shutdown():
                                print('shutdown ros')
                                return 'failure'
                return "success"



class BaseState(State):
        def __init__(self, robot, func, start_flight_state, goal_flight_state, timeout = 60):
                State.__init__(self, outcomes = ['success', 'failure'])
                self.robot = robot
                self.func = func
                self.start_flight_state = start_flight_state
                self.goal_flight_state = goal_flight_state
                self.timeout = timeout
        def execute(self, userdata):
                if self.robot.state != self.start_flight_state:
                        rospy.logwarn(self.__class__.__name__ + ': the robot state ({}) is not at the start_flight_state ({}).'.format(self.robot.state, self.start_flight_state))
                        return 'failure'
                self.func()
                self.start_t = rospy.get_time()
                while rospy.get_time() < self.start_t + self.timeout:
                        if self.robot.state == self.goal_flight_state:
                                return "success"
                        rospy.sleep(0.1)
                        rospy.logdebug_throttle(1.0, "current state: ({})".format(self.robot.state))
                        if rospy.is_shutdown():
                                print("shutdown ros")
                                return 'failure'
                rospy.logwarn('timeout')
                rospy.logwarn(self.__class__.__name__ + ': the robot state ({}) is not at the goal_flight_state ({}).'.format(self.robot.state, self.goal_flight_state))
                return "failure"

class Takeoff(BaseState):
        # 離陸
        def __init__(self, robot, func):   
                BaseState.__init__(self, robot, func, RobotState.START, RobotState.HOVER)

        def execute(self, userdata):
                if self.robot.state == RobotState.HOVER:
                        rospy.loginfo("The robot is already hovering.")
                        return 'success'
                return BaseState.execute(self, userdata)

class Land(BaseState):
        def __init__(self, robot, func):
                BaseState.__init__(self, robot, func, RobotState.HOVER, RobotState.STOP)
                        
class Approach(BaseState):
        def __init__(self, robot, func):
                BaseState.__init__(self, robot, func, RobotState.HOVER, RobotState.PALM_LAND_READY, timeout=180)

class PalmLand(BaseState):
        def __init__(self, robot, func):
                BaseState.__init__(self, robot, func, RobotState.PALM_LAND_READY, RobotState.STOP)
                
def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['success', 'failure'])
    flapper = Flapper()
    
    # Open the container
    with sm:
        # Add states to the container
        StateMachine.add('Start', Start(), 
                         transitions={'success': 'Takeoff', 'failure': 'failure'})
        StateMachine.add('Takeoff', Takeoff(flapper, flapper.takeoff), 
                         transitions={'success':'Approach', 'failure': 'failure'})
        StateMachine.add('Approach', Approach(flapper, flapper.approach),
                         transitions={'success': 'PalmLand', 'failure': 'failure'})
        StateMachine.add('PalmLand', PalmLand(flapper, flapper.palm_land),
                         transitions={'success': 'success', 'failure': 'failure'})

    # Execute SMACH plan
    outcome = sm.execute()
    if outcome == 'failure':
            flapper.stop()

if __name__ == '__main__':
        try:
                main()
        except Exception as e:
                print(traceback.format_exc())

