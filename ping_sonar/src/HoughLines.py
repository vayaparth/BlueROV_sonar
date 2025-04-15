#! /usr/bin/env python3

import cv2
import sys
import rospy 
import math 
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ping_sonar.msg import SonarEcho2

class Hough():
    def __init__(self):
        #CV Bridge
        self.bridge = CvBridge()

        #Subsribers
        rospy.Subscriber('/sonar/image',Image,self.HoughCallback)

        #Publishers
        self.HoughPub = rospy.Publisher('/sonar/hough',Image,10)

    #Function for creating lines and drawing on top of original image
    def HoughCallback(data,self):
        Temp_cv2_Img = self.bridge.imgmsg_to_cv2(data,mono8)
        canny_img = cv2.Canny(Temp_cv2_Img,50,200)
        lines = cv2.HoughLines(canny_img,1,np.pi/180,150)

        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

        Final_cv2_img = self.bridge.cv2_to_imgmsg(cdst,mono8)

        self.HoughPub.publish(Final_cv2_img)



if __name__ == "__main__":
    rospy.init_node("HoughLines")
    Hough()
    rospy.spin
