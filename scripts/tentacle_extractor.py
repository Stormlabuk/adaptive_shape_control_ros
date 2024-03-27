#!/usr/bin/env python
import rospy
import numpy as np
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import ximgproc
import matplotlib.pyplot as plt

class TentacleExtractor():
    def __init__(self) -> None:
        rospy.init_node('tentacle_extractor', anonymous=False)
        rospy.loginfo("TentacleExtractor node started")
        self.base_sub = rospy.Subscriber(
            '/base_img', Image, self.base_callback)
        self.bridge = CvBridge()
        self.tent_img = cv2.imread(
            "/home/vittorio/ros_ws/src/adaptive_ctrl/fake_tentacle/neutral.png")
        self.centre_line = []
        rospy.spin()

    def base_callback(self, msg):
        rospy.loginfo("base_callback")
        base_cv = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough")
        # base_cv = cv2.cvtColor(base_cv, cv2.COLOR_BGR2GRAY)
        tent_img = cv2.cvtColor(self.tent_img, cv2.COLOR_BGR2GRAY)
        base_cv = cv2.bitwise_not(base_cv)
        tent_only = cv2.bitwise_xor(base_cv, tent_img)
        tent = np.zeros_likes_like(tent_only)
        tent[tent_only == 255] = 255

        # at this point, tent_only holds the tentacle and a bit of noise
        filtered_tent = cv2.medianBlur(tent, 5)
        thinned = ximgproc.thinning(filtered_tent)
        points = np.argwhere(thinned == 255)
        # order the points based on distance from the origin
        points = points[np.argsort(np.linalg.norm(points, axis=1))]
        
        return




if __name__ == '__main__':
    try:
        TentacleExtractor()
    except rospy.ROSInterruptException:
        pass
