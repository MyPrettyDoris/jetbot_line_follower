#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#from jetbot import Camera
import cv2
import numpy as np

print(cv2.__version__)

dispW=640
dispH=480
flip=2

camSet='nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3264, height=2464, format=(string)NV12, framerate=(fraction)21/1 '\
    '! nvvidconv ! video/x-raw, width='+str(dispW)+', height = '+str(dispH)+', format=(string)BGRx '\
    '! videorate ! video/x-raw, framerate=(fraction)21/1 '\
    '! videoconvert ! appsink'

def main():
    rospy.init_node('camera_publisher')
    pub = rospy.Publisher('camera/image_raw', Image, queue_size = 1)
    bridge = CvBridge()
    cam=cv2.VideoCapture(camSet) #use JetBot camera API

    while not rospy.is_shutdown():
        ret, frame=cam.read()

        if ret:
            if  isinstance(frame, np.ndarray):
                ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                pub.publish(ros_image)
        
            else:
                print("frame is notnumpy array")
        else:
            print("camera read error")

        rospy.sleep(0.03)

    cam.release()

if __name__ == '__main__':
    main()