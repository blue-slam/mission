#!/usr/bin/env python

import yaml

import rospy
import tf

from smach_ros import SimpleActionState
from smach import State, StateMachine
from move_base_msgs.msg import MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from mission.msg import PlanResult, PlanFeedback


class _BuildPlan(State):
    def __init__(self, input_keys, output_keys):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'preempted'],
                       input_keys=input_keys,
                       output_keys=output_keys)

    def execute(self, userdata):
        try:
            rospy.loginfo('planning')

            plan_filename = userdata.plan_goal.plan_path
            rospy.loginfo('loading plan from: {}'.format(plan_filename))
            with open(plan_filename, 'r') as stream:
                userdata.waypoint_index = 0
                userdata.plan = yaml.load(stream)

            rospy.loginfo('adding {} waypoints using frame: {}'.format(len(plan_data['waypoints']), plan_data['frame']))
            # waypoints = []
            # for i, waypoint in enumerate(plan_data['waypoints']):
            #     rospy.loginfo('generating goal for waypoint {} at [{}, {}, {}]'.format(waypoint[0],
            #                                                                            waypoint[1],
            #                                                                            waypoint[2],
            #                                                                            waypoint[3]))
            #     quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, waypoint[3])
            #
            #     goal = MoveBaseGoal()
            #     goal.target_pose.header.frame_id = plan_data['frame']
            #     goal.target_pose.pose.position.x = waypoint[1]
            #     goal.target_pose.pose.position.y = waypoint[2]
            #     goal.target_pose.pose.position.z = 0.0
            #     goal.target_pose.pose.orientation.x = quaternion[0]
            #     goal.target_pose.pose.orientation.y = quaternion[1]
            #     goal.target_pose.pose.orientation.z = quaternion[2]
            #     goal.target_pose.pose.orientation.w = quaternion[3]
            #     waypoints.append(goal)

            return 'succeeded'

        except Exception as e:
            rospy.logerr(e.message)
            return 'aborted'


def _next_goal(userdata, goal):
    pass


def _goal_complete(userdata, status, result):
    pass


def create_plan_executor():
    pe = StateMachine(['succeeded', 'aborted', 'preempted'],
                      input_keys=['plan_goal'],
                      output_keys=['plan_result'])

    pe.userdata.plan_feedback = PlanFeedback()
    pe.userdata.waypoint_index = 0
    pe.userdata.waypoints = None
    keys = ['plan_index', 'plan', 'plan_goal', 'plan_result', 'plan_feedback']

    with pe:
        StateMachine.add('BUILD_PLAN', _BuildPlan(input_keys=keys, output_keys=keys),
                         transitions={'succeeded': 'RUN_PLAN', 'aborted': 'aborted', 'preempted': 'preempted'})
        StateMachine.add('RUN_PLAN', SimpleActionState('move_base', MoveBaseGoal,
                                                       goal_cb=_next_goal,
                                                       result_cb=_goal_complete,
                                                       input_keys=keys,
                                                       output_keys=keys,
                                                       outcomes=['next', 'succeeded', 'aborted', 'preempted']),
                         transitions={'next': 'RUN_PLAN', 'succeeded': 'succeeded', 'aborted': 'aborted',
                                      'preempted': 'preempted'})
    return pe
