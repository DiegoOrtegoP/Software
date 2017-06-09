#!/usr/bin/env python

import math

import rospy

from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped


class Joystick():

    def __init__(self):

        self.subscriber = rospy.Subscriber('/duckiebot/joy', Joy, self._process)

        self.publisher = rospy.Publisher('/duckiebot/joystick_orders', Twist2DStamped, queue_size=1)

        self.orders = Twist2DStamped()

        print('Running Joystick Node')
        print('--')
        print('Subscribed to topic /duckiebot/joy')
        print('--')
        print('Publishing on topic /duckiebot/joystick_orders')

    def _process(self, joystick_data):

        left_stick = joystick_data.axes[0]
        right_stick = joystick_data.axes[4]

        self.orders.omega = left_stick*10
        if joystick_data.axes[5] < 0:
            self.orders.v = right_stick*10
        else:
            self.orders.v = right_stick*0.5

        self.orders.header.stamp = rospy.get_rostime()

        self.publisher.publish(self.orders)


def main():

    rospy.init_node('Joystick')
    rospy.loginfo('Joystick')

    Joystick()

    rospy.spin()

if __name__ == '__main__':
    main()
