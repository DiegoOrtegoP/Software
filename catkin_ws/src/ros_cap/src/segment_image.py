#!/usr/bin/env python

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

#colores que funcionan:

lower_yellow = np.array([21,160,129])
upper_yellow = np.array([32,255,255])
lower_blue = np.array([110,30,30])
upper_blue = np.array([130,255,255])

#colores que no:

lower_red = np.array([0,50,50])
upper_red = np.array([36,255,255])
lower_green = np.array([54,50,50])
upper_green = np.array([109,255,255])

class SegmentImage():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber('/duckiebot/camera_node/image/raw', Image, self._process_image)

        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()
        
        self.pub = rospy.Publisher('/duckiebot/camera_node/segment_image/patos', Image, queue_size=1)


    def _process_image(self,img):

        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
        color_space = cv2.COLOR_BGR2HSV
        image_out = cv2.cvtColor(frame, color_space)

        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(image_out, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        segment_image = cv2.bitwise_and(frame,frame, mask= mask)
        segment_image_2 = self.bridge.cv2_to_imgmsg(segment_image, "bgr8")

        #Publicar imagenes
        self.pub.publish(segment_image_2)


def main():

    rospy.init_node('SegmentImage')

    SegmentImage()

    rospy.spin()

if __name__ == '__main__':
    main()
