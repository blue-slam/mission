import numpy as np
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from smach import State


class Teleop(State):

    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self._outcome = None
        self._cmd = Twist()
        max_rpm = rospy.get_param('BlueSlam/motors/max_rpm')
        r = rospy.get_param('BlueSlam/geometry/r')
        d = rospy.get_param('BlueSlam/geometry/d')
        self._max_vel = (np.pi * max_rpm * r) / 30
        self._max_omega = ((np.pi * max_rpm * r) / (15 * d)) * .50

    def execute(self, ud):

        try:
            self._outcome = None
            self._cmd.linear.x = 0
            self._cmd.angular.z = 0

            twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            joy_sub = rospy.Subscriber('joy', Joy, self._joy_handler)

            rate = rospy.Rate(50)
            while not rospy.is_shutdown() and self._outcome is None:
                twist_pub.publish(self._cmd)
                rate.sleep()

            return 'success'
        finally:
            self._cmd.linear.x = 0
            self._cmd.angular.z = 0
            twist_pub.publish(self._cmd)

    def _joy_handler(self, msg):
        if msg.buttons[1] or msg.buttons[3]:
            self._outcome = 'success'
        else:
            self._cmd.linear.x = msg.axes[5] * self._max_vel
            self._cmd.angular.z = msg.axes[0] * self._max_omega
