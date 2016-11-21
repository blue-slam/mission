import numpy as np
import rospy

import tf.transformations

from threading import Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from smach import State


class GotoRelativeHeading(State):

    def __init__(self, headings):
        State.__init__(self, outcomes=['success', 'canceled'])
        self._lock = Lock()
        self._goal_headings = headings
        self._odometry_pose = None
        self._outcome = None
        self._max_v = 0.25
        self._max_w = np.pi / 2
        self._kp = 1.0
        self._ki = 0.5
        self._kd = 1.0

    def execute(self, ud):

        self._odometry_pose = None
        self._outcome = None

        current_goal = 0
        target_heading = None
        last_error =0.0
        error = 0.0
        integral = 0.0
        dt = 1.0 / 50.0

        odometry_sub = rospy.Subscriber('odom', Odometry, self._odometry_handler)
        joy_sub = rospy.Subscriber('joy', Joy, self._joy_handler)
        twist_pub = rospy.Publisher('cmd_vel', Twist,   queue_size=10)

        rate = rospy.Rate(50)
        cmd = Twist()

        while not rospy.is_shutdown():

            if self._is_canceled():
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                twist_pub.publish(cmd)
                break

            if self._odometry_pose is not None:

                with self._lock:
                    pose = self._odometry_pose

                if target_heading is None:
                    target_heading = pose[2] + self._goal_headings[current_goal]
                    target_heading = np.arctan2(np.sin(target_heading), np.cos(target_heading))

                error = target_heading - pose[2]
                integral += error * dt
                derivative = (error - last_error) / dt
                last_error = error
                w = self._kp * error + self._ki * integral + self._kd * derivative

                print('goal: {:f}, theta: {:f}, error: {:f}, integral: {:f}, derivative: {:f} w: {:f}'.format(target_heading, pose[2], error, integral, derivative, w))

                if np.abs(error) <= np.pi / 180:
                    break

                cmd.linear.x = 0.0
                cmd.angular.z = w
                twist_pub.publish(cmd)

            rate.sleep()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        twist_pub.publish(cmd)

        if not self._is_canceled():
            return 'success'
        return self._outcome

    def _is_canceled(self):
        with self._lock:
            return self._outcome == 'canceled'

    def _joy_handler(self, msg):
        if msg.buttons[1]:
            with self._lock:
                self._outcome = 'canceled'

    def _odometry_handler(self, msg):
        quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        with self._lock:
            self._odometry_pose = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y], euler[2]])
