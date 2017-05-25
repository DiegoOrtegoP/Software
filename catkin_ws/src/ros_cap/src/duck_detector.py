#!/usr/bin/env python


import math
import rospy
# esa cosa
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped

# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV
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


class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber('/duckiebot/camera_node/image/raw', Image, self._process_image) 

        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 25

        self.image_publisher = rospy.Publisher('/duckiebot/camera_node/segment_image/patos', Image, queue_size=1)
        self.coordenadas_publisher = rospy.Publisher('/duckiebot/posicionPato', Point, queue_size=1)


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
        # Kernel y cosas

        kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        mask = cv2.erode(mask, kernel, iterations = 2)
        
        #Operacion morfologica dilate
        mask = cv2.dilate(mask, kernel, iterations = 5)

        image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        xmax=0
        ymax=0
        wmax=0
        hmax=0
        areamax=0
        for cnt in contours:
            #Obtener rectangulo
            x,y,w,h = cv2.boundingRect(cnt)
            area = w*h
            if area > areamax:
                areamax = area
                xmax=x
                ymax=y
                wmax=w
                hmax=h
            #Filtrar por area minima
            if w*h > self.min_area:

                #Dibujar un rectangulo en la imagen
                x1 = x
                y1 = y
                x2 = x+w
                y2 = y+h
                cv2.rectangle(frame, (x1,y1), (x2,y2), (255,0,255), 2)

        #Publicar frame
        imagen_final = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_publisher.publish(imagen_final)

        #Publicar Point center de mayor tamanio
        x = xmax
        y = ymax
        w = wmax
        h = hmax
        fx = 336.79653744960046
        fy = 336.0126772357778
        ancho_pato = 4
        alto_pato = 3.5
        center_x = x+(w/2)
        center_y = y+(h/2)
        if w == 0:
            center_z = 1000000
        else:
            center_z = (fx*ancho_pato)/w
        cero = (imagen_final.width/2)
        pos_rel = center_x - cero
        rate = rospy.Rate(30)
        msg = Point()
        msg.x = center_x
        msg.y = center_y
        msg.z = center_z
        self.coordenadas_publisher.publish(msg)
        rate.sleep()
        
def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
