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



        self.phantom_low_p = rospy.get_param("phantom_low_p", [0,0,143])
        self.phantom_high_p = rospy.get_param("phantom_high_p", [180,59,255])
        self.inserter_low_p = rospy.get_param("inserter_low_p", [28,144,82])
        self.inserter_high_p = rospy.get_param("inserter_high_p", [151,255,156])
        self.phantom_low_ = np.array(self.phantom_low_p)
        self.phantom_high_ = np.array(self.phantom_high_p)
        self.inserter_low_ = np.array(self.inserter_low_p)
        self.inserter_high_ = np.array(self.inserter_high_p)
        self.cam_width = rospy.get_param("cam_width", 1124)
        self.cam_height = rospy.get_param("cam_height", 1040)
        self.cam_offset_x = rospy.get_param("cam_offset_x", 284)
        self.cam_offset_y = rospy.get_param("cam_offset_y", 74)
        self.bridge = CvBridge()
        self.initial_pubs = rospy.Service("initial_imgproc", SetBool, self.initial_image_processing)
        self.phantom_pub = rospy.Service("phantom_imgproc", SetBool, self.phantom_image_processing)
        self.publish_maps = True
        self.publish_phantom = True
        rospy.init_node('image_processor', anonymous=False)
        rospy.spin()

    def initial_image_processing(self, req):
        self.publish_maps = req.data
        res = SetBoolResponse(success=True, message="Initial image processing started")
        return res
    
    def phantom_image_processing(self, req):
        self.publish_phantom = req.data
        res = SetBoolResponse(success=True, message="Phantom processing started")
        return res

    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_resize = image[:self.cam_width, 0:self.cam_height]
            image_resize = cv2.resize(image_resize, (600, 600))
        except CvBridgeError as e:
            rospy.logerr(e)
        # convert to hsv
        image_hsv = cv2.cvtColor(image_resize, cv2.COLOR_BGR2HSV)
        phantom = cv2.inRange(image_hsv, self.phantom_low_, self.phantom_high_)
        inserter = cv2.inRange(image_hsv, self.inserter_low_, self.inserter_high_)

        # phantom_blur = cv2.GaussianBlur(phantom, (5, 5), 0)
        phantom_erode = cv2.erode(phantom, None, iterations=1)
        phantom_erode = cv2.dilate(phantom_erode, None, iterations=1)
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

        base_img = cv2.add(inserter, phantom)

        bridge = CvBridge()
        try:
            self.image_pub.publish(bridge.cv2_to_imgmsg(image_resize, "bgr8"))
            if(self.publish_maps):
                self.inserter_pub.publish(bridge.cv2_to_imgmsg(
                    inserter, "mono8"))
                self.phantom_pub.publish(bridge.cv2_to_imgmsg(
                    phantom, "mono8"))
                self.base_pub.publish(bridge.cv2_to_imgmsg(
                    base_img, "mono8"))
                self.publish_maps = False
            if(self.publish_phantom):
                self.phantom_pub.publish(bridge.cv2_to_imgmsg(
                    phantom, "mono8"))
                self.publish_phantom = False
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == "__main__":
    try:
        ImageProcessor()
    except rospy.ROSInterruptException:
        pass
