#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
class GridProcessor:
    def __init__(self) -> None:

        self.phantom_sub = rospy.Subscriber('/phantom_img', Image, self.phantom_callback)
        self.base_sub = rospy.Subscriber('/base_img', Image, self.base_callback)

        self.costmap_pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=1)
        self.occ_grid_pub = rospy.Publisher('/grid', OccupancyGrid, queue_size=1)
        
        self.phantom = Image()
        self.base = Image()
        self.bridge = CvBridge()
        rospy.init_node('grid_processor', anonymous=False)
        rospy.spin()

    def phantom_callback(self, msg):
        # self.phantom = msg
        phantom_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        phantom_cv = cv2.dilate(phantom_cv, element, iterations=2)
        vals = np.array(phantom_cv.tolist())
        vals = vals / 255 * 100
        vals = vals.astype(np.uint8)
        self.phantom = self.bridge.cv2_to_imgmsg(vals, encoding="passthrough")
        self.process_costmap()
        return

    def base_callback(self, msg):
        base_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        # inserter_cv = cv2.dilate(inserter_cv, element, iterations=1)
        vals = np.array(base_cv.tolist())
        vals = vals / 255 * 100
        vals = vals.astype(np.uint8)
        self.base = self.bridge.cv2_to_imgmsg(vals, encoding="passthrough")
        self.process_occgrid()
        return

    def process_costmap(self):
        # process costmap here
        costmap = OccupancyGrid()
        costmap.header.stamp = rospy.Time.now()
        costmap.header.frame_id = "map"
        costmap.info.resolution = 1
        costmap.info.width = 600
        costmap.info.height = 600
        costmap.info.origin.position.x = 0
        costmap.info.origin.position.y = 0
        costmap.info.origin.position.z = 0
        costmap.info.origin.orientation.x = 0
        costmap.info.origin.orientation.y = 0
        costmap.info.origin.orientation.z = 0
        costmap.info.origin.orientation.w = 1
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
        grid.info.width = 600
        grid.info.height = 600
        grid.info.origin.position.x = 0
        grid.info.origin.position.y = 0
        grid.info.origin.position.z = 0
        grid.info.origin.orientation.x = 0
        grid.info.origin.orientation.y = 0
        grid.info.origin.orientation.z = 0
        grid.info.origin.orientation.w = 1
        grid.data = self.base.data
        data = []
        # if(self.phantom is not None):
        for i in range(600*600):
            data.append(self.base.data[i] | self.phantom.data[i])
        # else:
            # data = self.base.data
        grid.data = data
        self.occ_grid_pub.publish(grid)
        return


if __name__ == "__main__":
    GridProcessor()