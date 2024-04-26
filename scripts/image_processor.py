#!/usr/bin/env python
import rospy
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String
import matplotlib.pyplot as plt 

class ImageProcessor():
    def __init__(self) -> None:

        # self.img_path_sub = rospy.Subscriber(
        #     "img_path", String, self.image_callback)
        self_live_img_sub = rospy.Subscriber(
            "/pylon_camera_node/image_rect", Image, self.image_callback)

        self.inserter_pub = rospy.Publisher(
            "inserter_img", Image, queue_size=10)
        self.base_pub = rospy.Publisher("base_img", Image, queue_size=10)
        self.phantom_pub = rospy.Publisher("phantom_img", Image, queue_size=10)
        self.image_pub = rospy.Publisher("image", Image, queue_size=10)

        self.initial_pubs = rospy.Service("initial_imgproc", SetBool, self.initial_image_processing)
        self.publish_maps = True

        self.phantom_low_p = rospy.get_param("~phantom_low_p", (90,0,100))
        self.phantom_high_p = rospy.get_param("~phantom_high_p", (180,100,185))
        self.inserter_low_p = rospy.get_param("~phantom_low_p", (0,0,0))
        self.inserter_high_p = rospy.get_param("~phantom_high_p", (131,212,87))

        self.phantom_low_ = self.phantom_low_p
        self.phantom_high_ = self.phantom_high_p
        self.inserter_low_ = self.inserter_low_p
        self.inserter_high_ = self.inserter_high_p
        self.bridge = CvBridge()
        rospy.init_node('image_processor', anonymous=False)
        rospy.spin()

    def initial_image_processing(self, req):
        self.publish_maps = req.data
        res = SetBoolResponse(success=True, message="Initial image processing started")
        return res

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

        base_img = cv2.add(self.inserter, self.phantom)

        bridge = CvBridge()
        try:
            self.image_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
            if(self.publish_maps):
                self.inserter_pub.publish(bridge.cv2_to_imgmsg(
                    self.inserter, "passthrough"))
                self.phantom_pub.publish(bridge.cv2_to_imgmsg(
                    self.phantom, "passthrough"))
                self.base_pub.publish(bridge.cv2_to_imgmsg(
                    base_img, "passthrough"))
                # self.publish_maps = False
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == "__main__":
    try:
        ImageProcessor()
    except rospy.ROSInterruptException:
        pass
