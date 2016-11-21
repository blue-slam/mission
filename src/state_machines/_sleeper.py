from time import sleep

import rospy
from smach import State


class Sleeper(State):
    def __init__(self, name, duration):
        State.__init__(self, outcomes=['success'])
        self._name = name
        self._duration = duration

    def execute(self, ud):
        rospy.loginfo(self._name)
        sleep(self._duration)
        return 'success'
