#!/usr/bin/env python
import rospy
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


class ImageProcessor():
    def __init__(self) -> None:
        self.inserterLowVals = rospy.get_param(
            '~inserter_low_vals', [45, 0, 0])
        self.inserterHighVals = rospy.get_param(
            '~inserter_high_vals', [150, 255, 255])
        self.phantomLowVals = rospy.get_param('~phantom_low_vals', [0, 0, 95])
        self.phantomHighVals = rospy.get_param(
            '~phantom_high_vals', [180, 86, 206])
        self.img_path_sub = rospy.Subscriber(
            "img_path", String, self.image_callback)
        self.inserter_pub = rospy.Publisher(
            "inserter_img", Image, queue_size=10)
        self.phantom_pub = rospy.Publisher("phantom_img", Image, queue_size=10)
        self.inserter = np.empty((0, 0), dtype=np.uint8)
        self.phantom = np.empty((0, 0), dtype=np.uint8)
        rospy.init_node('image_processor', anonymous=False)
        rospy.spin()

    def image_callback(self, data):
        img = cv2.imread(data.data, cv2.IMREAD_COLOR)

        self.inserter = np.zeros_like(img)
        self.phantom = np.zeros_like(img)
        hsv_img = cv2.GaussianBlur(img, (5, 5), 0)
        structuring_elem = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        hsv_img = cv2.erode(hsv_img, structuring_elem, iterations=1)
        hsv_img = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2HSV)
        inserter = cv2.inRange(hsv_img, np.array(
            self.inserterLowVals), np.array(self.inserterHighVals))
        phantom = cv2.inRange(hsv_img, np.array(
            self.phantomLowVals), np.array(self.phantomHighVals))
        phantom[inserter > 0] = 0
        # Find contours of phantom
        phantom_contours, _ = cv2.findContours(phantom, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find contours of inserter
        inserter_contours, _ = cv2.findContours(inserter, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # # Create blank image for drawing contours
        # contour_img = np.zeros_like(img)
        # # Draw contours of phantom in blue
        # cv2.drawContours(contour_img, phantom_contours, -1, (255, 0, 0), 2)
        # # Draw contours of inserter in red
        # cv2.drawContours(contour_img, inserter_contours, -1, (0, 0, 255), 2)


        phantom_contours = sorted(phantom_contours, key=cv2.contourArea, reverse=True)
        inserter_contours = sorted(inserter_contours, key=cv2.contourArea, reverse=True)
        # top_cnt_disp = np.zeros_like(img)
        # cv2.drawContours(top_cnt_disp, phantom_contours[:5], -1, (255, 0, 0), -1)
        # cv2.drawContours(top_cnt_disp, inserter_contours[:1], -1, (0, 0, 255), -1)
        # cv2.imshow("Top contours", top_cnt_disp)
        # cv2.waitKey(0)

        self.phantom = cv2.drawContours(
            self.phantom, phantom_contours[:2], -1, 255, 3)
        self.inserter = cv2.drawContours(
            self.inserter, inserter_contours[:1], -1, 255, 3)
        bridge = CvBridge()
        try:
            self.inserter_pub.publish(bridge.cv2_to_imgmsg(
                self.inserter, "passthrough"))
            self.phantom_pub.publish(bridge.cv2_to_imgmsg(
                self.phantom, "passthrough"))
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == "__main__":
    try:
        ImageProcessor()
    except rospy.ROSInterruptException:
        pass
