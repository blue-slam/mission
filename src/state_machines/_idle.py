import rospy
from smach import State
from sensor_msgs.msg import Joy


class Idle(State):
    def __init__(self):
        State.__init__(self, outcomes=['B0', 'B1', 'B2'])

    def execute(self, ud):
        outcome = None
        while outcome is None:
            msg = rospy.wait_for_message('joy', Joy)
            if msg.buttons[0]:
                outcome = 'B0'
            elif msg.buttons[3]:
                outcome = 'B1'
            elif msg.buttons[2]:
                outcome = 'B2'
        return outcome
