#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from nav2_msgs.action import NavigateToPose

import yasmin
from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_ros import ActionState

TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = "/scan"

ANG_DER = 30.0 * math.pi / 180.0
ANG_IZQ = -ANG_DER


# ============================================================
#  ESTADOS NAV2
# ============================================================
class GotoWPState(ActionState):
    def __init__(self, wp):
        self.wp = wp
        self.gh = None
        super().__init__(
            NavigateToPose,
            "/navigate_to_pose",
            create_goal_handler=self.create_goal_handler,
            feedback_handler=self.feedback_handler
        )

    def create_goal_handler(self, blackboard):
        pose = Pose()
        pose.position.x = self.wp[0]
        pose.position.y = self.wp[1]
        pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose.pose = pose
        goal.pose.header.frame_id = "map"

        return goal
    
    def feedback_handler(self, blackboard, feedback):
        print(f"Quedan {feedback.distance_remaining} metros")


# ============================================================
#  WAIT ENTER
# ============================================================
class WaitForEnterState(State):
    """Estado que espera hasta que el usuario pulse ENTER."""
    def __init__(self):
        super().__init__(outcomes={'succeeded'})

    def execute(self, blackboard):
        print("\n--- Esperando: Pulse ENTER para continuar ---\n")
        input()
        return 'succeeded'


# ============================================================
#  MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Node("project")

    # FSM YASMIN
    sm = StateMachine(outcomes={'end'})
    wait = WaitForEnterState()
    estado1 = GotoWPState([0.0, 0.0])
    estado2 = GotoWPState([-10.0, 0.0])
    estado3 = GotoWPState([-10.0, -10.0])
    estado4 = GotoWPState([0.0, -10.0])
    estado5 = GotoWPState([10.0, 10.0])
    
    estados_cancelables = [wait, estado1, estado2, estado3, estado4, estado5]
    '''
    sm.add_state('wait',   wait, transitions={'succeeded': 'WP1'})
    sm.add_state('WP1', estado1, transitions={'succeeded': 'WP2'})
    sm.add_state('WP2', estado2, transitions={'succeeded': 'WP3'})
    sm.add_state('WP3', estado3, transitions={'succeeded': 'WP4'})
    sm.add_state('WP4', estado4, transitions={'succeeded': 'WP5'})
    sm.add_state('WP5', estado1, transitions={'succeeded': 'end'})
    '''
    sm.add_state('wait',   wait, transitions={'succeeded': 'WP1'})
    sm.add_state('WP1', estado5, transitions={'succeeded': 'WP2'})
    sm.add_state('WP2', estado2, transitions={'succeeded': 'end'})
    #'''
    
    sm.set_start_state('wait')
    sm.validate()

    try:
        outcome = sm.execute(Blackboard())
    except KeyboardInterrupt:
        for state in estados_cancelables:
            state.cancel_state()
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
