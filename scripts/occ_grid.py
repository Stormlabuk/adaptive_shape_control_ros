#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from shapeforming_msgs.srv import GetInsertion
import cv2
import sys, os
import numpy as np

class OccGrid:
    def __init__(self):
        self.bridge = CvBridge()

        # Publishers
        self.grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.costmap_pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=1)

        # Service Servers
        # self.grid_srv = rospy.Service('/srv_grid', Trigger, self.trig_grid)
        # self.map_srv = rospy.Service('/srv_map', Trigger, self.trig_map)
    
        # Subscribers
        self.phantomSub = rospy.Subscriber('/phantom_img', Image, self.phantom_callback)
        self.imgSub = rospy.Subscriber('/img', Image, self.img_callback)
        
    def phantom_callback(self, msg):
        ## receives an Image message, pushes it out with costmap_pub
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
        img = cv2.dilate(img, element, iterations=1)
        vals = np.array(img.tolist())
        width = vals.shape[1]
        height = vals.shape[0]
        vals = vals / 255 * 100
        vals = vals.astype(np.uint8)
        img = self.bridge.cv2_to_imgmsg(vals, encoding="passthrough")

        costmap = OccupancyGrid()
        costmap.header.stamp = rospy.Time.now()
        costmap.header.frame_id = "map"
        costmap.info.resolution = 1
        costmap.info.width = width
        costmap.info.height = height
        costmap.info.origin.position.x = 0
        costmap.info.origin.position.y = 0
        costmap.info.origin.position.z = 0
        costmap.info.origin.orientation.x = 0
        costmap.info.origin.orientation.y = 0
        costmap.info.origin.orientation.z = 0
        costmap.info.origin.orientation.w = 1
        costmap.data = img.data
        rospy.loginfo("Publishing costmap")
        self.costmap_pub.publish(costmap)
        return
    
    def img_callback(self, msg):
        ## receives an Image message, pushes it out with grid_pub
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))
        # img = cv2.dilate(img, element, iterations=1)
        vals = np.array(img.tolist())
        width = vals.shape[1]
        height = vals.shape[0]
        vals = vals / 255 * 100
        vals = vals.astype(np.uint8)
        img = self.bridge.cv2_to_imgmsg(vals, encoding="passthrough")

        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = rospy.Time.now()
        occ_grid.header.frame_id = "map"
        occ_grid.info.resolution = 1
        occ_grid.info.width = width
        occ_grid.info.height = height
        occ_grid.info.origin.position.x = 0
        occ_grid.info.origin.position.y = 0
        occ_grid.info.origin.position.z = 0
        occ_grid.info.origin.orientation.x = 0
        occ_grid.info.origin.orientation.y = 0
        occ_grid.info.origin.orientation.z = 0
        occ_grid.info.origin.orientation.w = 1
        occ_grid.data = img.data
        rospy.loginfo("Publishing occupancy grid")
        self.grid_pub.publish(occ_grid)
        return

def main(args):
    rospy.init_node('occ_map_node')
    occ_grid = OccGrid()
    rospy.spin()
    return 0

if __name__ == '__main__':
    main(sys.argv)