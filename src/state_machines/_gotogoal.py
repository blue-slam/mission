import numpy as np
import rospy
import tf.transformations

from threading import Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from smach import State


class GotoGoal(State):

    def __init__(self, waypoint):
        State.__init__(self, outcomes=['success', 'canceled'])
        self._lock = Lock()
        self._goal_pose = np.array(waypoint)[np.newaxis].T
        self._odometry_pose = np.array([3, 1])
        self._outcome = None
        self._cr = rospy.get_param('BlueSlam/geometry/cr')
        self._kp = 3
        self._ka = 8
        self._kb = -1.5
        self._max_v = .25
        self._max_w = 25

    def execute(self, ud):

        self._odometry_pose = None
        self._outcome = None

        odometry_sub = rospy.Subscriber('odom', Odometry, self._odometry_handler)
        joy_sub = rospy.Subscriber('joy', Joy, self._joy_handler)
        twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

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

                delta_pos = self._goal_pose[0:2, :] - pose[0:2, :]

                polar = np.zeros([3, 1])
                polar[0, 0] = np.sqrt(np.sum(np.square(delta_pos)))
                polar[1, 0] = -pose[2, 0] + np.arctan2(delta_pos[1, 0], delta_pos[0, 0])
                polar[2, 0] = -pose[2, 0] - polar[1, 0]

                if np.abs(polar[0, 0]) > self._cr:
                    cmd.linear.x = min(self._kp * polar[0, 0], self._max_v)
                    cmd.angular.z = min(self._ka * polar[1, 0] + self._kb * polar[2, 0], self._max_w)
                    twist_pub.publish(cmd)
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    twist_pub.publish(cmd)
                    break

            rate.sleep()

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
            self._odometry_pose = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [euler[2]]])
