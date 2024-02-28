#!/usr/bin/env python
import rospy
import os, sys
from std_msgs.msg import String
import cv2

class ImageFetcher():
    def __init__(self):
        rospy.init_node('image_fetcher', anonymous=False)
        self.image_path_pub = rospy.Publisher("img_path", String, queue_size=10)
        self.rate = rospy.Rate(0.5)  # 1 Hz
        default_path = "/home/vittorio/ros_ws/src/adaptive_ctrl/inserter"
        path = rospy.get_param('~image_base_path', default_path)
        img_paths = os.listdir(path)
        img_paths.sort()
        self.img_paths = [os.path.join(path, img_path) for img_path in img_paths]

        self.fetch_image()

    def fetch_image(self):
        while not rospy.is_shutdown():
            # Fetch the image
            for img_path in self.img_paths:
            # img_path = self.img_paths[0]
                self.image_path_pub.publish(img_path)
                self.rate.sleep()


if __name__ == '__main__':
    try:
        ImageFetcher()
    except rospy.ROSInterruptException:
        pass