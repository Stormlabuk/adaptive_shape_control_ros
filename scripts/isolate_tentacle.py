#!/usr/bin/env python
import rospy
import cv2
import matplotlib.pyplot as plt
import numpy as np
from skimage.morphology import skeletonize
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from cv_bridge import CvBridge, CvBridgeError



class IsolateTentacle():
    def __init__(self) -> None:
        self.live_img_sub = rospy.Subscriber(
            "/image", Image, self.image_callback)
        self.base_img_sub = rospy.Subscriber(
            "/inserter_img", Image, self.base_image_callback)
        self.imgproc_call = rospy.ServiceProxy("initial_imgproc", SetBool)
        self.tent_img_pub = rospy.Publisher(
            "tentacle_img", Image, queue_size=10)

        self.hsv_low = rospy.get_param("inserter_low_p", (28, 144, 82))
        self.hsv_high = rospy.get_param("inserter_high_p", (151, 255, 156))
        # rospy.loginfo("HSV low: " + str(self.hsv_low))
        # rospy.loginfo("HSV high: " + str(self.hsv_high))
        self.hsv_low = np.array(self.hsv_low)
        self.hsv_high = np.array(self.hsv_high)

        self.base_image = None
        self.live_image = None
        self.bridge = CvBridge()
        self.base_image_found = False

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
            image_hsv = cv2.GaussianBlur(image_hsv, (15, 15), 0)
        except CvBridgeError as e:
            rospy.logerr(e)
        if (self.base_image_found):
            tent_inserter = cv2.inRange(image_hsv, self.hsv_low, self.hsv_high)
            tent_inserter = cv2.erode(tent_inserter, None, iterations=2)
            tent_inserter = cv2.dilate(tent_inserter, None, iterations=10)
            tent_only = cv2.bitwise_xor(tent_inserter, self.base_image)
            tent_only = cv2.erode(tent_only, None, iterations=8)
            skeleton = skeletonize(tent_only)
            skeleton = skeleton.astype(np.uint8) * 255
            self.tent_img_pub.publish(self.bridge.cv2_to_imgmsg(skeleton, "mono8"))
        else:
            rospy.logwarn("Base image not found")


    def base_image_callback(self, data):
        self.base_image=self.bridge.imgmsg_to_cv2(data, "mono8")
        self.base_image_found=True


if __name__ == "__main__":
    rospy.init_node('isolate_tentacle', anonymous=False)
    isolate_tentacle=IsolateTentacle()
    rospy.spin()
