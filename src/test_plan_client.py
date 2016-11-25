#!/usr/bin/env python

import rospy
import actionlib
from mission.msg import PlanAction, PlanGoal, PlanResult


def handle_feedback(feedback):
    print('feedback: {}'.format(feedback.goal_name))


rospy.init_node('plan_action_client')
client = actionlib.SimpleActionClient('mission/plan', PlanAction)
client.wait_for_server()
goal = PlanGoal()
goal.plan_path = '/home/stephen/Workspace/blue-slam/robot/src/mission/params/plan.yaml'
client.send_goal(goal, feedback_cb=handle_feedback)
client.wait_for_result()
if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
    print('status: {} duration: {}'.format(client.get_state(), client.get_result().time_elapsed.to_sec()))
else:
    print('status: {}'.format(client.get_state()))
