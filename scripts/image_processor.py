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

        self.img_path_sub = rospy.Subscriber(
            "img_path", String, self.image_callback)
        self.inserter_pub = rospy.Publisher(
            "inserter_img", Image, queue_size=10)
        self.phantom_pub = rospy.Publisher("phantom_img", Image, queue_size=10)
        self.image_pub = rospy.Publisher("image", Image, queue_size=10)
        self.inserter = np.empty((0, 0), dtype=np.uint8)
        self.phantom = np.empty((0, 0), dtype=np.uint8)
        rospy.init_node('image_processor', anonymous=False)
        rospy.spin()

    def image_callback(self, data):
        img = cv2.imread(data.data, cv2.IMREAD_COLOR)

        # 1. Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 2. Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # 3. Binary thresholding
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
        # 4. find contours
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 5. sort contours by surface area
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        # 6. Draw the top 2 contours. That is the phantom image
        self.phantom = np.zeros_like(gray)
        # self.phantom = cv2.cvtColor(self.phantom, cv2.COLOR_BGR2GRAY)
        self.phantom = cv2.drawContours(
            self.phantom, contours[:2], -1, 255, -1)
        # 7. Draw the bounding rect of the phantom, use as a mask for the inserter
        merged_contour = cv2.findNonZero(self.phantom)
        box = cv2.boundingRect(merged_contour)
        self.inserter = gray.copy()
        cv2.rectangle(self.inserter, box, (0), cv2.FILLED)
        # 8. Canny edge the inserter
        inserter_th = cv2.Canny(self.inserter, 180, 255)
        # 9. Dilate the edges
        inserter_dilated = cv2.dilate(inserter_th, None, iterations=1)
        # 10. Find contours on inserter
        contours, _ = cv2.findContours(
            inserter_dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # 11. Sort by surface area
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        # 12. Draw the convex Hull of the top contour

        hull = cv2.convexHull(contours[0])
        self.inserter = np.zeros_like(self.inserter)
        self.inserter = cv2.drawContours(
            self.inserter, [hull], -1, 255, cv2.FILLED)

        bridge = CvBridge()
        try:
            # rospy.loginfo("Publishing images")
            self.image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
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
