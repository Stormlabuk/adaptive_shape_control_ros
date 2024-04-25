#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt



class ThresholdCal():
    def __init__(self) -> None:
        self_live_img_sub = rospy.Subscriber(
            "/pylon_camera_node/image_rect", Image, self.image_callback)
        self.phantom_pub = rospy.Publisher("phantom_img", Image, queue_size=10)
        self.inserter_pub = rospy.Publisher(
            "inserter_img", Image, queue_size=10)

        self.phantom_hsv_low = rospy.Subscriber(
            "phantom_hsv_low", Vector3, self.phantom_hsv_low_callback)
        
        self.phantom_hsv_high = rospy.Subscriber(
            "phantom_hsv_high", Vector3, self.phantom_hsv_high_callback)
        
        
        # self.inserter_hsv_low = rospy.Subscriber(
        #     "inserter_hsv_low", Vector3, self.inserter_hsv_low_callback)
        
        # self.inserter_hsv_high = rospy.Subscriber(
        #     "inserter_hsv_high", Vector3, self.inserter_hsv_high_callback)

        self.phantom_low_ = (90,0,100)
        self.phantom_high_ = (180,100,185)
        self.inserter_low_ = (0,0,0)
        self.inserter_high_ = (131,212,87)
        self.bridge = CvBridge()

        rospy.spin()

    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            image_resize = image[:, 250:1400]
            image_resize = cv2.resize(image_resize, (600, 600))
        except CvBridgeError as e:
            rospy.logerr(e)
        # convert to hsv
        image_hsv = cv2.cvtColor(image_resize, cv2.COLOR_BGR2HSV)
        phantom = cv2.inRange(image_hsv, self.phantom_low_, self.phantom_high_)
        inserter = cv2.inRange(image_hsv, self.inserter_low_, self.inserter_high_)

        # phantom_blur = cv2.GaussianBlur(phantom, (5, 5), 0)
        phantom_erode = cv2.erode(phantom, None, iterations=2)
        phantom_erode = cv2.dilate(phantom_erode, None, iterations=3)
        # 4. find contours
        ph_contours, _ = cv2.findContours(
            phantom_erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 5. sort contours by surface area
        ph_contours = sorted(ph_contours, key=cv2.contourArea, reverse=True)
        disp = np.zeros_like(phantom)
        cv2.drawContours(disp, ph_contours[:3], -1, 255, -1)
        phantom = disp

        inserter_erode = cv2.erode(inserter, None, iterations=1)
        inserter_erode = cv2.dilate(inserter_erode, None, iterations=6)
        # 4. find contours
        in_contours, _ = cv2.findContours(
            inserter_erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 5. sort contours by surface area
        in_contours = sorted(in_contours, key=cv2.contourArea, reverse=True)
        disp = np.zeros_like(inserter)
        cv2.drawContours(disp, in_contours[:3], -1, 255, -1)
        inserter = disp

        self.phantom_pub.publish(self.bridge.cv2_to_imgmsg(phantom, "mono8"))
        self.inserter_pub.publish(self.bridge.cv2_to_imgmsg(inserter, "mono8"))

    def phantom_hsv_low_callback(self, data):
        self.phantom_low_ = (data.x, data.y, data.z)

    def phantom_hsv_high_callback(self, data):
        self.phantom_high_ = (data.x, data.y, data.z)
    
    def inserter_hsv_low_callback(self, data):
        self.inserter_low_ = (data.x, data.y, data.z)

    def inserter_hsv_high_callback(self, data):
        self.inserter_high_ = (data.x, data.y, data.z)


if __name__ == "__main__":
    rospy.init_node('threshold_cal', anonymous=False)
    ThresholdCal()
