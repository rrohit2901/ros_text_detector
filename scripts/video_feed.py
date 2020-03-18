#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


rospy.init_node("camera",anonymous = True)
pub = rospy.Publisher("video_feed", Image, queue_size = 1)
rate = rospy.Rate(10)
bridge = CvBridge()
cap = cv2.VideoCapture(0)
while not rospy.is_shutdown():
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    messg = bridge.cv2_to_imgmsg(frame, "bgr8")
    pub.publish(messg)
    cv2.imshow("webcam", frame)
    cv2.waitKey(100)
    rate.sleep()