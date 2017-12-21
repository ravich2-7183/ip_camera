#!/usr/bin/env python
import sys
import cv2
import roslib
import rospy
import numpy as np
import argparse
import requests
from std_msgs.msg import Bool
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 

class IPCameraListener(object):
    def __init__(self, url):
        self.vcap = cv2.VideoCapture()
        ret = self.vcap.open(url)
        
        self.vcap_source = ''
        if ret == True:
            self.vcap_source = 'cv2'
            print 'Using cv2.VideoCapture'
        else:
            self.vcap_source = 'requests'
            self.vcap = requests.get(url, stream=True)
            print 'Reading directly from http image stream'
        
        print 'Successfully opened ip camera listener...images will be published on receipt of img_processed messages'
        
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("img_processed", Bool, self.callback)
        self.bridge = CvBridge()
    
    def callback(self, data):
        if self.vcap_source == 'cv2':
            ret, frame = self.vcap.read()
        else: # vcap_source == 'requests'
            bytes = ''
            while True:
                bytes += self.vcap.raw.read(1024) # TODO search only in the current 1024 block
                a = bytes.find('\xff\xd8') # jpg begin symbol # TODO check for 'unlucky' coincidences
                if a != -1:
                    bytes = bytes[a:]
                    break
            while True:
                bytes += self.vcap.raw.read(1024) # TODO search only in the current 1024 block
                b = bytes.find('\xff\xd9') # jpg end symbol # TODO check for 'unlucky' coincidences
                if b != -1: # and (b-a) >= 640*480*3: # TODO remove assumption that images are color VGA
                    break
            jpg = bytes[0:b+2]
            frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        img_msg.header.stamp = rospy.get_rostime()
        self.image_pub.publish(img_msg)
    
    def close():
        if self.vcap_source == 'cv2':
            self.vcap.release()

if __name__ == '__main__':
    # Usage:
    # $ python ip_camera_listener -u YOUR_IP_CAMERA_URL
    # Listens for Bool messages on the /img_processed topic,
    # and in response publishes an Image message on the /camera/image_raw topic, 
    # reading from the http url supplied as the first argument.
    
    parser = argparse.ArgumentParser(prog='ip_camera.py', description='reads a given url string and dumps it to a ros_image topic')
    parser.add_argument('-u', '--url', default='http://192.168.43.1:8080/video', help='camera stream url to parse')
    args = parser.parse_args(rospy.myargv()[1:])
    
    rospy.init_node('ip_camera_listener', anonymous=True)
    
    ip_camera_listener = IPCameraListener(args.url)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        ip_camera_listener.close()

