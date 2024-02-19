#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
import os, sys


def main():
    img_paths = []
    base_paths = '/home/vittorio/ros_ws/src/adaptive_ctrl/inserter'
    img_paths = os.listdir(base_paths) 
    # sort img_paths alphabetically
    img_paths.sort()
    img_paths = [os.path.join(base_paths, img_path) for img_path in img_paths]
    print(img_paths)

    rospy.init_node('image_path_finder')
    path_pub = rospy.Publisher('/image_path', String, queue_size=10)
    while(rospy.is_shutdown() == False):
        for img_path in img_paths:
            path_pub.publish(img_path)
            rospy.sleep(1)
    return


if __name__ == "__main__":
    main()