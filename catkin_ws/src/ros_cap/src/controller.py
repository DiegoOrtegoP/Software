#!/usr/bin/env python


import math
import rospy
# esa cosa
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped



from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse



import numpy as np

class Controller():

    def __init__(self):

        self.joy_subscriber = rospy.Subscriber('/duckiebot/possible_cmd', Twist2DStamped, self._process) 
        self.pos_subscriber = rospy.Subscriber('/duckiebot/posicionPato', Point, )
        self.wheels_publisher = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)


    def _process(self,cmd,pos):
        msg = Twist2DStamped()
        if pos.z <= 10:
            msg.omega = 0
            msg.v = 0
        else:
            msg.omega = cmd.omega
            msg.v = cmd.v
        self.wheels_publisher.publish(msg)
        
def main():

    rospy.init_node('Controller')

    Controller()

    rospy.spin()

if __name__ == '__main__':
    main()
