#!/usr/bin/env python


import math
import rospy
# esa cosa
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped



from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

distancia_minima = 60

import numpy as np

class Controller():

    def __init__(self):

        self.joy_subscriber = rospy.Subscriber('/duckiebot/possible_cmd', Twist2DStamped, self._process) 
        self.pos_subscriber = rospy.Subscriber('/duckiebot/posicionPato', Point, self.guardar_posicion)
        self.wheels_publisher = rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=1)
        self.posicion = 100


    def guardar_posicion(self,dist):
        self.posicion = dist.z


    def _process(self,mov):
        msg = Twist2DStamped()
        if self.posicion <= distancia_minima:
            msg.omega = mov.omega
            msg.v = 0
        else:
            msg.omega = mov.omega
            msg.v = mov.v
        self.wheels_publisher.publish(msg)
        
def main():

    rospy.init_node('Controller')

    Controller()

    rospy.spin()

if __name__ == '__main__':
    main()
