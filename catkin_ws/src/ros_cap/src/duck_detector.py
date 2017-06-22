#!/usr/bin/env python


import math
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy

from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

# Colors:

lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])
lower_orange = np.array([10, 50, 50])
upper_orange = np.array([20, 255, 255])
lower_yellow = np.array([20, 150, 130])
upper_yellow = np.array([35, 255, 255])
lower_green = np.array([35, 50, 50])
upper_green = np.array([100, 255, 255])
lower_blue = np.array([100, 50, 50])
upper_blue = np.array([130, 255, 255])
lower_purple = np.array([130, 50, 50])
upper_purple = np.array([179, 255, 255])

# Variables:

dimensions_kernel = 5
iterations_erode = 2
iterations_dilate = 2

area_min = 30

read_data = True    # Whether you'd like to read the data file with the camera's updated focal distance values
data_location = '/home/duckiebot/duckietown/catkin_ws/src/duckietown/config/baseline/calibration/camera_intrinsic/duckiebot.yaml'
fx_over_fy_relation = 0.5   # In what proportion you'd like to use the focal distances from the x and y axes

duck_width = 4.0    # Centimeters
duck_height = 3.5   # Centimeters

print_position = True   # Prints the coordinates of the duck
only_z = True   # Prints z coordinate only


class DuckDetector():

    def __init__(self):

        self.image_subscriber = rospy.Subscriber('/duckiebot/camera_node/image/rect', Image, self._process_image)

        self.image_publisher = rospy.Publisher('/duckiebot/camera_node/image/rect/ducks', Image, queue_size=1)
        self.coordinates_publisher = rospy.Publisher('/duckiebot/duck_point', Point, queue_size=1)

        self.bridge = CvBridge()

        self.kernel = np.ones((dimensions_kernel, dimensions_kernel), np.uint8)

        self.cv_image = Image()
        self.point = Point()

        if read_data:
            calibration = open(data_location, 'r')
            i = 0
            self.fx = ''
            self.fy = ''
            for linea in calibration:
                if i == 2:
                    a = 0
                    for l in linea:
                        if l == ',':
                            a = 0
                        if a == 1:
                            self.fx += l
                        if l == '[':
                            a = 1
                if i == 3:
                    a = 1
                    for l in linea:
                        if l == ',':
                            a = 0
                        if a == 1:
                            self.fy += l
            calibration.close()
            self.fx = float(self.fx)
            self.fy = float(self.fy)
        else:
            self.fx = 336.79653744960046
            self.fy = 336.01267723577780

        print('Running DuckDetector Node')
        print('--')
        print('Subscribed to topic /duckiebot/camera_node/image/rect')
        print('--')
        print('Publishing on topic /duckiebot/camera_node/image/rect/ducks')
        print('Publishing on topic /duckiebot/duck_point')

    def _process_image(self, img):
        
        # Change image format
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Create mask
        mask = cv2.inRange(cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV), lower_yellow, upper_yellow)

        # Morphological operations on mask
        mask = cv2.erode(mask, self.kernel, iterations=iterations_erode)
        mask = cv2.dilate(mask, self.kernel, iterations=iterations_dilate)

        # Parameters biggest rectangle
        x_max = 0
        y_max = 0
        w_max = 0
        h_max = 0
        area_max = 0

        # Rectangles data
        image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            # Parameters on the rectangle
            x, y, w, h = cv2.boundingRect(cnt)
            area = w*h

            # Filter by minimum area
            if area > area_min:
                cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (80, 20, 77), 2)

                # Find biggest rectangle
                if area > area_max:
                    area_max = area
                    x_max = x
                    y_max = y
                    w_max = w
                    h_max = h

        # Publish image with rectangles
        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))

        # Position of the duck in the image
        self.point.x = x_max + w_max/2.0
        self.point.y = y_max + h_max/2.0
        try:
            z_x = (self.fx * duck_width * h_max) / area_max
            z_y = (self.fy * duck_height * w_max) / area_max
            self.point.z = fx_over_fy_relation * z_x + (1 - fx_over_fy_relation) * z_y
        except ZeroDivisionError:
            self.point.z = float('Inf')

        # Publish position
        self.coordinates_publisher.publish(self.point)

        # Print position values
        if print_position:
            print('--')
            if not only_z:
                print('x= '+str(self.point.x))
                print('y= '+str(self.point.y))
            print('z= '+str(self.point.z))


def main():

    rospy.init_node('DuckDetector')

    DuckDetector()

    rospy.spin()

if __name__ == '__main__':
    main()
