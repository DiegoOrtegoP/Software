#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
import math


def main():
    rospy.init_node('test_subscriber_and_publisher')
    rospy.loginfo('test_subscriber_and_publisher')
    sub = rospy.Subscriber('/duckiebot/joy', Joy, process_callback)
    rospy.spin()


def process_callback(msg):
    base_pub = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
    x = msg.axes[0]
    y = msg.axes[4]
    msg2 = Twist2DStamped()
    msg2.header.stamp = rospy.get_rostime()     
    msg2.omega = x*10
    if msg.axes[5]<0:
      msg2.v = y*10
    else:
      msg2.v = y*0.5
    base_pub.publish(msg2)
if __name__ == '__main__':
    main()

