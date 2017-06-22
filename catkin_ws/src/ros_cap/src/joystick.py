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

        left_stick_x = joystick_data.axes[0]
        # left_stick_y = joystick_data.axes[1]
        # left_trigger = joystick_data.axes[2]
        # right_stick_x = joystick_data.axes[3]
        right_stick_y = joystick_data.axes[4]
        # right_trigger = joystick_data.axes[5]
        # arrows_x = joystick_data.axes[6]
        # arrows_y = joystick_data.axes[7]
        # a = joystick_data.buttons[0]
        # b = joystick_data.buttons[1]
        # x = joystick_data.buttons[2]
        # y = joystick_data.buttons[3]
        # left_bumper = joystick_data.buttons[4]
        # right_bumper = joystick_data.buttons[5]
        # back = joystick_data.buttons[6]
        # start = joystick_data.buttons[7]
        # xbox = joystick_data.buttons[8]
        # left_stick_press = joystick_data.buttons[9]
        # right_stick_press = joystick_data.buttons[10]
        # arrow_left = joystick_data.buttons[11]
        # arrow_right = joystick_data.buttons[12]
        # arrow_up = joystick_data.buttons[13]
        # arrow_down = joystick_data.buttons[14]

        turn = left_stick_x
        advance = right_stick_y

        print(turn)
        print(advance)

        self.orders.omega = turn*10
        if joystick_data.axes[5] < 0:
            self.orders.v = advance*10
        else:
            self.orders.v = advance*0.5

        self.orders.header.stamp = rospy.get_rostime()

        self.publisher.publish(self.orders)


def main():

    rospy.init_node('Joystick')
    rospy.loginfo('Joystick')

    Joystick()

    rospy.spin()

if __name__ == '__main__':
    main()
