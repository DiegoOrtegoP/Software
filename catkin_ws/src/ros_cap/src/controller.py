#!/usr/bin/env python


import math
import numpy as np

import rospy

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

# Variables

distance_min = 70


class Controller():

    def __init__(self):

        self.position_subscriber = rospy.Subscriber('/duckiebot/duck_point', Point, self.save_position)
        self.joystick_subscriber = rospy.Subscriber('/duckiebot/joystick_orders', Twist2DStamped, self._process)

        self.wheels_publisher = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)

        self.distance = float('Inf')
        self.movement = Twist2DStamped()

        print('Running Controller Node')
        print('--')
        print('Subscribed to topic /duckiebot/duck_point')
        print('Subscribed to topic /duckiebot/joystick_orders')
        print('--')
        print('Publishing on topic /duckiebot/wheels_driver_node/car_cmd')

    def save_position(self, position):
        self.distance = position.z

    def _process(self, orders):
        self.movement.omega = orders.omega
        if self.distance <= distance_min:
            self.movement.v = 0
        else:
            self.movement.v = orders.v

        self.wheels_publisher.publish(self.movement)


def main():

    rospy.init_node('Controller')

    Controller()

    rospy.spin()

if __name__ == '__main__':
    main()
