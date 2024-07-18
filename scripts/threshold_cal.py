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
        
        self.image_resize_pub = rospy.Publisher(
            "image_resize", Image, queue_size=10)

        self.inserter_hsv_low = rospy.Subscriber(
            "inserter_hsv_low", Vector3, self.inserter_hsv_low_callback)
        
        self.inserter_hsv_high = rospy.Subscriber(
            "inserter_hsv_high", Vector3, self.inserter_hsv_high_callback)

        self.phantom_low_ = (15,0,165)
        self.phantom_high_ = (180,88,255)
        self.inserter_low_ = (80,179,16)
        self.inserter_high_ = (180,255,255)
        self.bridge = CvBridge()

        self.cam_width = rospy.get_param("cam_width", 1124)
        self.cam_height = rospy.get_param("cam_height", 1040)
        self.cam_offset_x = rospy.get_param("cam_offset_x", 284)
        self.cam_offset_y = rospy.get_param("cam_offset_y", 74)

        rospy.spin()

    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_resize = image[:self.cam_height, 0:self.cam_width]
            # image_resize = cv2.resize(image_resize, (600, 600))
        except CvBridgeError as e:
            rospy.logerr(e)
        # convert to hsv
        image_hsv = cv2.cvtColor(image_resize, cv2.COLOR_BGR2HSV)
        phantom = cv2.inRange(image_hsv, self.phantom_low_, self.phantom_high_)
        inserter = cv2.inRange(image_hsv, self.inserter_low_, self.inserter_high_)

        phantom_blur = cv2.GaussianBlur(phantom, (11, 11), 0)
        phantom_erode = cv2.erode(phantom_blur, None, iterations=2)
        phantom_erode = cv2.dilate(phantom_erode, None, iterations=3)

        # 4. find contours
        ph_contours, _ = cv2.findContours(
            phantom_erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 5. sort contours by surface area
        ph_contours = sorted(ph_contours, key=cv2.contourArea, reverse=True)
        disp = np.zeros_like(phantom)
        cv2.drawContours(disp, ph_contours[:3], -1, 255, -1)
        phantom = disp

        inserter = cv2.GaussianBlur(inserter, (5, 5), 0)
        inserter_erode = cv2.erode(inserter, None, iterations=1)
        inserter_erode = cv2.dilate(inserter_erode, None, iterations=6)
        # 4. find contours
        in_contours, _ = cv2.findContours(
            inserter_erode, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 5. sort contours by surface area
        in_contours = sorted(in_contours, key=cv2.contourArea, reverse=True)
        disp = np.zeros_like(inserter)
        cv2.drawContours(disp, in_contours[:1], -1, 255, -1)
        inserter = disp

        self.phantom_pub.publish(self.bridge.cv2_to_imgmsg(phantom, "mono8"))
        self.inserter_pub.publish(self.bridge.cv2_to_imgmsg(inserter, "mono8"))
        self.image_resize_pub.publish(self.bridge.cv2_to_imgmsg(image_resize, "bgr8"))

    def phantom_hsv_low_callback(self, data):
        self.phantom_low_ = (data.x, data.y, data.z)
        rospy.loginfo(f"Phantom low: {self.phantom_low_}")

    def phantom_hsv_high_callback(self, data):
        self.phantom_high_ = (data.x, data.y, data.z)
        rospy.loginfo(f"Phantom high: {self.phantom_high_}")
    
    def inserter_hsv_low_callback(self, data):
        self.inserter_low_ = (data.x, data.y, data.z)
        rospy.loginfo(f"Inserter low: {self.inserter_low_}")

    def inserter_hsv_high_callback(self, data):
        self.inserter_high_ = (data.x, data.y, data.z)
        rospy.loginfo(f"Inserter high: {self.inserter_high_}")


if __name__ == "__main__":
    rospy.init_node('threshold_cal', anonymous=False)
    ThresholdCal()
