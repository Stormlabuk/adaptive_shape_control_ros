#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt


class ThresholdCal():
    def __init__(self) -> None:
        self_live_img_sub = rospy.Subscriber(
            "/pylon_camera_node/image_rect", Image, self.image_callback)
        self.phantom_pub = rospy.Publisher("phantom_img", Image, queue_size=10)
        self.inserter_pub = rospy.Publisher(
            "inserter_img", Image, queue_size=10)

        self.phantom_th_sub = rospy.Subscriber(
            "phantom_th", Int32, self.phantom_th_callback)
        self.inserter_th_sub = rospy.Subscriber(
            "inserter_th", Int32, self.inserter_th_callback
        )

        self.phantom_th = 105
        self.inserter_th = 50
        self.bridge = CvBridge()
        rospy.spin()

    def image_callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "passthrough")
            img = img[:, 250:1400]
            img = cv2.resize(img, (600, 600))
        except CvBridgeError as e:
            rospy.logerr(e)
        # 1. Convert to grayscale
        if (len(img.shape) > 2):
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        else:
            gray = img

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # 3. Binary thresholding
        _, thresh = cv2.threshold(
            blurred, self.phantom_th, 255, cv2.THRESH_BINARY)
        # plt.imshow(thresh)

        # 4. find contours
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 5. sort contours by surface area
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        phantom = np.zeros_like(gray)
        # 6. draw contours
        phantom = cv2.drawContours(phantom, contours[:3], -1, 255, -1)
        self.phantom_pub.publish(self.bridge.cv2_to_imgmsg(phantom, "mono8"))

        merged_contour = cv2.findNonZero(phantom)
        # box = cv2.boundingRect(merged_contour)
        inserter = gray.copy()
        # remove all merged_contour from inserter
        for i in range(merged_contour.shape[0]):
            inserter[merged_contour[i][0][1], merged_contour[i][0][0]] = 0

        inserter[:, 250:600] = 0
        inserter_th = cv2.threshold(
            inserter, self.inserter_th, 255, cv2.THRESH_BINARY)[1]
        # plt.imshow(inserter_th, cmap='gray')
        # plt.show()
        # 9. Dilate the edges
        inserter_dilated = cv2.dilate(inserter_th, None, iterations=1)
        # # 10. Find contours on inserter
        contours, _ = cv2.findContours(
            inserter_dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # # 11. Sort by surface area
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        # hull = cv2.convexHull(contours[0])
        inserter = np.zeros_like(inserter)
        inserter = cv2.drawContours(inserter, contours[:1], -1, 255, -1)
        self.inserter_pub.publish(self.bridge.cv2_to_imgmsg(inserter, "mono8"))

    def phantom_th_callback(self, data):
        self.phantom_th = data.data

    def inserter_th_callback(self, data):
        self.inserter_th = data.data


if __name__ == "__main__":
    rospy.init_node('threshold_cal', anonymous=False)
    ThresholdCal()
