#!/usr/bin/env python
import cv2
import urllib 
import numpy as np
from sensor_msgs.msg import Image 
import roslib
import sys
import rospy
import cv
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 

class IPCameraListener(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("ip_camera",Image,self.callback)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e

        (cols,rows) = cv.GetSize(cv_image)
        if cols > 60 and rows > 60 :
            cv.Circle(cv_image, (50,50), 10, 255)

        cv.ShowImage("IP_camera Listener", cv_image)
        cv.WaitKey(3)
        
if __name__ == '__main__':
  ip_camera_listener = IPCameraListener()
  rospy.init_node('ip_camera_listener', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()
