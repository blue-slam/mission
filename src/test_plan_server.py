#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult


def do_move_base(goal):
    rospy.sleep(2)
    result = MoveBaseResult()
    server.set_succeeded(result)

rospy.init_node('test_plan_server')
server = actionlib.SimpleActionServer('move_base', MoveBaseAction, do_move_base, False)
server.start()
rospy.spin()
