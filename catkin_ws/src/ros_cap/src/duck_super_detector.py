#!/usr/bin/env python

import math
import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,0,0])
upper_red = np.array([0,0,0])
lower_yellow = np.array([20,150,130])
upper_yellow = np.array([35,255,255])


class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber("/duckiebot/camera_node/image/raw",Image,self._process_image) 

        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 30

        #publisher
        self.pub = rospy.Publisher("/duckiebot/patofiltrado",Image,queue_size=1)
        self.publito = rospy.Publisher("/duckiebot/punto",Point,queue_size=1)
        print("explotando en 3, 2, 1...") 

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
        #segment_image = cv2.bitwise_and(frame,frame, mask= mask)
        #imga= self.bridge.cv2_to_imgmsg(segment_image, "bgr8")
        #self.pub.publish(imga)

        kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        img_out = cv2.erode(mask, kernel, iterations = 1)
        
        #Operacion morfologica dilate
        img_out = cv2.dilate(img_out, kernel, iterations = 1)

        

        image, contours, hierarchy = cv2.findContours(img_out,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        x1=0
        x2=0
        y1=0
        y2=0
        for cnt in contours:
            #Obtener rectangulo
            x,y,w,h = cv2.boundingRect(cnt)

            #Filtrar por area minima
            if w*h > self.min_area:


                #Dibujar un rectangulo en la imagen
                x1=x
                y1=y
                x2=x+w
                y2=y+h
                frame=cv2.rectangle(frame, (x1,y1), (x2,y2), (80,20,77), 2)
                #Publicar Point center de mayor tamanio
                puntillo=Point()
                puntillo.x=((x1+x2)/2)
                puntillo.y=((y1+y2)/2)
                #Foco respecto a X fx truncado
                puntillo.z=(310.089*3.5/w)
                #foco respecto a Y fy truncado
                #puntillo.z=(309.71*3.5/sqrt(y1^2+y2^2))
                self.publito.publish(puntillo)
        
        #Publicar frame
        #imagesita=cv2.cvtColor(rectangle,cv2.COLOR_GRAY2BGR)
        imgb= self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.pub.publish(imgb)
       
def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
