#!/usr/bin/env python
import rospy
import cv2
import matplotlib.pyplot as plt
import numpy as np
from skimage.morphology import skeletonize, remove_small_objects
from plantcv import plantcv as pcv
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveRequest, DiscretiseCurveResponse
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Vector3


class IsolateTentacle():
    def __init__(self) -> None:
        self.live_img_sub = rospy.Subscriber(
            "/image", Image, self.image_callback)
        self.base_img_sub = rospy.Subscriber(
            "/inserter_img", Image, self.base_image_callback)
        self.imgproc_call = rospy.ServiceProxy("initial_imgproc", SetBool)
        self.skel_pub_flag = rospy.Service(
            "publish_skeleton", SetBool, self.skel_pub_callback)

        self.discretise_call = rospy.ServiceProxy(
            "obv_discretise_curve", DiscretiseCurve)

        self.tent_img_pub = rospy.Publisher(
            "tentacle_img", Image, queue_size=10)

        self.ins_point_sub = rospy.Subscriber(
            "/insertion_point", Point, self.insertion_point_callback)
        self.insertion_point = Point(0, 0, 0)
        self.hsv_low = rospy.get_param("tentacle_low_p", (28, 144, 82))
        self.hsv_high = rospy.get_param("tentacle_high_p", (151, 255, 156))

        rospy.logwarn("Tentacle HSV low: " + str(self.hsv_low))
        rospy.logwarn("Tentacle HSV high: " + str(self.hsv_high))

        # self.hsv_low = np.array([0,0,0])
        # self.hsv_high = np.array([132,138,255])
        self.mm_pixel = rospy.get_param("mm_pixel", 5)
        self.link_l_mm = rospy.get_param("precomputation/len", 10)

        self.link_l_mm = float(self.link_l_mm)
        self.pixel_mm = 1/self.mm_pixel
        self.link_px = self.link_l_mm * self.pixel_mm
        # rospy.loginfo("HSV low: " + str(self.hsv_low))
        # rospy.loginfo("HSV high: " + str(self.hsv_high))
        self.hsv_low = np.array(self.hsv_low)
        self.hsv_high = np.array(self.hsv_high)

        self.base_image = None
        self.live_image = None
        self.bridge = CvBridge()
        self.base_image_found = False
        self.pub_skeleton = True

        rospy.wait_for_service("initial_imgproc")
        req = SetBoolRequest()
        req.data = True
        self.imgproc_call(req)
        rospy.spin()

    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # Apply some blur
            # image_hsv = cv2.GaussianBlur(image_hsv, (15, 15), 0)
        except CvBridgeError as e:
            rospy.logerr(e)
        if (self.base_image_found):

            tent_inserter = cv2.inRange(image_hsv, self.hsv_low, self.hsv_high)
            tent_inserter = cv2.GaussianBlur(tent_inserter, (3, 3), 0)
            tent_inserter = cv2.erode(tent_inserter, None, iterations=1)
            tent_inserter = cv2.dilate(tent_inserter, None, iterations=6)

            kernel = np.ones((5, 5), np.uint8)
            # closing = cv2.morphologyEx(tent_inserter, cv2.MORPH_CLOSE, kernel)
            # opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
            tent_cnt, _ = cv2.findContours(
                tent_inserter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            tent_cnt = sorted(tent_cnt, key=cv2.contourArea, reverse=True)
            tent_dsp = np.zeros_like(tent_inserter)
            tent_inserter = cv2.drawContours(
                tent_dsp, tent_cnt[:1], -1, 255, -1)
            # tent_inserter = cv2.dilate(tent_inserter,
            #                           None, iterations=1)
            tent_inserter_f = tent_dsp

            tent_only = cv2.bitwise_and(
                tent_dsp, cv2.bitwise_not(self.base_image))
            tent_only = cv2.morphologyEx(tent_only, cv2.MORPH_CLOSE, kernel)
            tent_only = cv2.morphologyEx(tent_only, cv2.MORPH_OPEN, kernel)
            # tent_only = cv2.blur(tent_only, (5, 5))

            skeleton = skeletonize(tent_only)
            skeleton = skeleton.astype(np.uint8) * 255

            skeleton[0:int(self.insertion_point.y),
                     0:int(self.insertion_point.x)] = 0

            skeleton_clean, _, seg_obj = pcv.morphology.prune(
                skeleton, size=20)

            # To find the longest object in segmented_obj:
            # Initialize maximum length and corresponding object
            max_length = 0
            longest_obj = None

            if len(seg_obj) != 0:
                # Analyze each object to find the longest one
                for obj in seg_obj:
                    # Calculate the distance of each object (assuming segmented_obj are the contours)
                    length = cv2.arcLength(obj, closed=False)
                    if length > max_length:
                        max_length = length
                        longest_obj = obj
                if(max_length > 0):
                    # Draw the longest object
                    timg = np.zeros_like(skeleton_clean)
                    cv2.drawContours(timg, [longest_obj], -1, 255, -1)
                    # cv2.imshow("Longest Object", timg)
                    # cv2.waitKey(1)

                    if (self.pub_skeleton):
                        self.tent_img_pub.publish(
                            self.bridge.cv2_to_imgmsg(timg, "mono8"))
        else:
            rospy.logwarn("Base image not found")

    def base_image_callback(self, data):
        self.base_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        self.base_image_found = True
        self.base_image = cv2.dilate(self.base_image,
                                     None, iterations=2)

    def insertion_point_callback(self, data):
        self.insertion_point = data
        return

    def skel_pub_callback(self, req):
        self.pub_skeleton = req.data
        res = SetBoolResponse(
            success=True, message="Skeleton publishing started")
        return res


if __name__ == "__main__":
    rospy.init_node('isolate_tentacle', anonymous=False)
    isolate_tentacle = IsolateTentacle()

    while (not rospy.is_shutdown()):
        rospy.spin()
