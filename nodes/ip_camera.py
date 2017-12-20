#!/usr/bin/env python
import sys
import cv2
import argparse
import roslib
import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 

class IPCamera(object):
    def __init__(self, url):
        try:
            self.vcap = cv2.VideoCapture(url)
        except:
            rospy.logerr('Unable to open ip camera stream: ' + str(url))
            sys.exit()
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
        self.bridge = CvBridge()

if __name__ == '__main__':
    # Usage:
    # $ python ip_camera -u YOUR_IP_CAMERA_URL -g
    parser = argparse.ArgumentParser(prog='ip_camera.py', description='reads a given url string and dumps it to a ros_image topic')
    parser.add_argument('-g', '--gui', action='store_true', help='show a GUI of the camera stream')
    parser.add_argument('-u', '--url', default='http://192.168.43.1:8080/video', help='camera stream url to parse')
    args = parser.parse_args(rospy.myargv()[1:])
    
    rospy.init_node('ip_camera', anonymous=True)
    
    print 'Opening ip camera'
    ip_camera = IPCamera(args.url)
    print 'Successfully opened ip camera'
    
    while not rospy.is_shutdown() and ip_camera.vcap.isOpened():
        ret, frame = ip_camera.vcap.read()
        if ret == False:
            print 'Could not read frame'
            break
        
        img_msg = ip_camera.bridge.cv2_to_imgmsg(frame, "bgr8")
        img_msg.header.stamp = rospy.get_rostime()
        ip_camera.image_pub.publish(img_msg)
        
        if args.gui:
            cv2.imshow('IP Camera', frame)
            if cv2.waitKey(1) == 27:
                break

    ip_camera.vcap.release()
    cv2.destroyAllWindows()
