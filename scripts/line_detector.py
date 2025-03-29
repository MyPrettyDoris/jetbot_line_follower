#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from jetbot_line_follower.msg import LineError

class LineDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.error_pub = rospy.Publisher('/line_error', LineError, queue_size=1)
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

	# 白底黑线的HSV阈值（黑色线的范围）
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([255, 255, 80])

    def image_callback(self, msg):
        try:
	    # 转换ROS图像为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 水平翻转图像（因为摄像头朝后）
            #cv_image = cv2.flip(cv_image, 1)  # 1表示水平翻转
            
            # 裁剪ROI（只保留图像下方1/3区域，靠近车尾的地面）
            height, width = cv_image.shape[:2]
            roi = cv_image[int(height*0.85):height, :]  # 从70%高度到底部

	    # 转换为HSV空间并提取黑色线
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_black, self.upper_black)

	    # 降噪处理
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	    #cv2.imshow("Mask", mask)
	    #cv2.waitKey(1)

	    # 检测轮廓并计算中线偏差
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                max_contours = max(contours, key=cv2.contourArea)
                M = cv2.moments(max_contours)
                cx = int(M['m10'] / (M['m00'] + 1e-5))  # 避免除零
                frame_center = width // 2
                error = cx - frame_center  # 偏差=中线-图像中心
	    else:
                error = 0  # 未检测到线时偏差设为0
            
            # 发布偏差
            error_msg = LineError()
            error_msg.error = error
            self.error_pub.publish(error_msg)

        except Exception as e:
            rospy.logerr("Image processing error: %s" % str(e))

if __name__ == '__main__':
    rospy.init_node('line_detector')
    detector = LineDetector()
    rospy.spin()
