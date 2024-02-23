#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from shapeforming_msgs.srv import GetInsertion, GetInsertionRequest
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np


class FindInsertionPoint:
    def __init__(self) -> None:
        self.inserter_img = np.empty((0, 0), dtype=np.uint8)

        self.inserter_sub = rospy.Subscriber(
            "inserter_img", Image, self.inserter_callback)

        self.insertion_point_pub = rospy.Publisher(
            "insertion_point", Point, queue_size=10)
        self.insertion_point_marker = rospy.Publisher(
            "insertion_point_marker", Marker, queue_size=10)

    def inserter_callback(self, msg):
        bridge = CvBridge()
        self.inserter_img = bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough")
        self.find_inserter_tip()
        return

    def find_inserter_tip(self):
        insertion_point = Point()

        contour, _ = cv2.findContours(
            self.inserter_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contour) > 0:
            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)
            approx = approx.reshape(approx.shape[0], approx.shape[2])


        self.insertion_point_pub.publish(insertion_point)
        return


if __name__ == "__main__":
    rospy.init_node('find_insertion_point', anonymous=False)
    FindInsertionPoint()
    rospy.spin()
