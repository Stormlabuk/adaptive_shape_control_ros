#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import tf_conversions
import tf2_ros

class GridProcessor:
    def __init__(self) -> None:

        self.phantom_sub = rospy.Subscriber('/phantom_img', Image, self.phantom_callback)
        self.base_sub = rospy.Subscriber('/base_img', Image, self.base_callback)

        self.costmap_pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=1)
        self.occ_grid_pub = rospy.Publisher('/grid', OccupancyGrid, queue_size=1)
        self.width = 0
        self.height = 0
        self.cam_width = rospy.get_param("cam_width", 1124)
        self.cam_height = rospy.get_param("cam_height", 1040)
        self.phantom = Image()
        self.base = Image()
        self.bridge = CvBridge()
        rospy.init_node('grid_processor', anonymous=False)
        rospy.spin()

    def phantom_callback(self, msg):
        # self.phantom = msg
        phantom_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        phantom_cv = cv2.dilate(phantom_cv, element, iterations=2)
        vals = np.array(phantom_cv.tolist())
        vals = vals / 255 * 100
        vals = vals.astype(np.uint8)
        self.phantom = self.bridge.cv2_to_imgmsg(vals, encoding="mono8")
        self.width = self.phantom.width
        self.height = self.phantom.height
        self.process_costmap()
        return

    def base_callback(self, msg):
        base_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        # element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        # inserter_cv = cv2.dilate(inserter_cv, element, iterations=1)
        vals = np.array(base_cv.tolist())
        vals = vals / 255 * 100
        vals = vals.astype(np.uint8)
        self.base = self.bridge.cv2_to_imgmsg(vals, encoding="mono8")
        self.width = self.base.width
        self.height = self.base.height
        self.process_occgrid()
        return

    def process_costmap(self):
        # process costmap here
        costmap = OccupancyGrid()
        costmap.header.stamp = rospy.Time.now()
        costmap.header.frame_id = "map"
        costmap.info.resolution = 1
        costmap.info.width = self.width
        costmap.info.height = self.height
        costmap.info.origin.position.x = 0
        costmap.info.origin.position.y = 0
        costmap.info.origin.position.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(np.pi, 0, 0)
        costmap.info.origin.orientation.x = q[0]
        costmap.info.origin.orientation.y = q[1]
        costmap.info.origin.orientation.z = q[2]
        costmap.info.origin.orientation.w = q[3]
        # costmap.data = np.zeros((600*600, 1), dtype=np.uint8).flatten().tolist()

        costmap.data = self.phantom.data
        # if(self.inserter_data is not None):
        #     costmap.data = np.logical_or(costmap.data, self.inserter_data)
        

        self.costmap_pub.publish(costmap)
        return
    
    def process_occgrid(self):
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"
        grid.info.resolution = 1
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = 0
        grid.info.origin.position.y = 0
        grid.info.origin.position.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(np.pi, 0, 0)
        grid.info.origin.orientation.x = q[0]
        grid.info.origin.orientation.y = q[1]
        grid.info.origin.orientation.z = q[2]
        grid.info.origin.orientation.w = q[3]
        grid.data = self.base.data

        

        # grid.data = data
        self.occ_grid_pub.publish(grid)
        return


if __name__ == "__main__":
    GridProcessor()