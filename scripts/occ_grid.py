#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
import cv2
import sys, os
import numpy as np

class OccGrid:
    """
    Represents an occupancy grid.

    Attributes:
        bridge (CvBridge): OpenCV bridge for image conversion.
        img_pub (rospy.Publisher): Publisher for debug image.
        map_pub (rospy.Publisher): Publisher for occupancy grid map.
        grid_pub (rospy.Publisher): Publisher for occupancy grid.
        costmap_pub (rospy.Publisher): Publisher for costmap.
        grid_srv (rospy.Service): Service for grid triggering.
        map_srv (rospy.Service): Service for map triggering.
        phantomSub (rospy.Subscriber): Subscriber for phantom image.
        imgSub (rospy.Subscriber): Subscriber for image.
        img_path (str): Path to the image file.
        img (numpy.ndarray): Image array.
        width (int): Width of the image.
        height (int): Height of the image.
        img_gray (numpy.ndarray): Grayscale image array.
        grid (OccupancyGrid): Occupancy grid.
        cost_map (OccupancyGrid): Costmap.
    """

    def __init__(self):
        """
        Initializes the OccGrid object.
        """
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher('/img_debug', Image, queue_size=1)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.grid_pub = rospy.Publisher('/grid', OccupancyGrid, queue_size=1)
        self.costmap_pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=1)
        self.grid_srv = rospy.Service('/srv_grid', Trigger, self.trig_grid)
        self.map_srv = rospy.Service('/srv_map', Trigger, self.trig_map)
        self.phantomSub = rospy.Subscriber('/phantom_img', Image, self.phantom_callback)
        self.imgSub = rospy.Subscriber('/img', Image, self.img_callback)
        self.img_path = '/home/vittorio/ros_ws/src/adaptive_ctrl/img/disp_cnt.png'
        self.img = cv2.imread(self.img_path, cv2.IMREAD_GRAYSCALE)
        if(self.img is None):
            print("Image not found")
            sys.exit()
        self.width = self.img.shape[0]
        self.height = self.img.shape[1]
        
        # convert img to binary

        self.img_gray = self.img.copy()
        elem = cv2.getStructuringElement(cv2.MORPH_RECT, (11,11))
        self.img_gray = cv2.dilate(self.img_gray, elem)
        gray_vals = np.array(self.img_gray.tolist())
        gray_vals = gray_vals / 255.0 * 100
        self.img_gray = gray_vals.astype(np.uint8)


        self.img[self.img > 0] = 100
        self.img = self.bridge.cv2_to_imgmsg(self.img, encoding="passthrough")
        self.img_gray = self.bridge.cv2_to_imgmsg(self.img_gray, encoding="passthrough")
        self.grid = OccupancyGrid()
        self.cost_map = OccupancyGrid()
        self.img2occ_grid()
        self.img2costmap()


    def phantom_callback(self, msg):
        """
        Callback function for phantom subscriber.

        Args:
            msg (Image): Image message.

        """
        self.img = msg
        self.img = self.bridge.imgmsg_to_cv2(self.img, desired_encoding="passthrough")
        self.img[self.img > 0] = 100
        self.img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        elem = cv2.getStructuringElement(cv2.MORPH_RECT, (11,11))
        self.img_gray = cv2.dilate(self.img_gray, elem)
        gray_vals = np.array(self.img_gray.tolist())
        gray_vals = gray_vals / 255.0 * 100
        self.img_gray = gray_vals.astype(np.uint8)
        self.img_gray = self.bridge.cv2_to_imgmsg(self.img_gray, encoding="passthrough")
        self.img2costmap()
        self.publish_map()

    def img_callback(self, msg):
        """
        Callback function for image subscriber.

        Args:
            msg (Image): Image message.

        """
        self.img = msg
        self.img = self.bridge.imgmsg_to_cv2(self.img, desired_encoding="passthrough")
        self.img[self.img > 0] = 100
        # self.img = self.bridge.cv2_to_imgmsg(self.img, encoding="passthrough")
        # self.img_gray = self.bridge.imgmsg_to_cv2(self.img, desired_encoding="passthrough")
        # # elem = cv2.getStructuringElement(cv2.MORPH_RECT, (11,11))
        # # self.img_gray = cv2.dilate(self.img_gray, elem)
        # # gray_vals = np.array(self.img_gray.tolist())
        # # gray_vals = gray_vals / 255.0 * 100
        # self.img_gray = gray_vals.astype(np.uint8)
        self.img_gray = self.bridge.cv2_to_imgmsg(self.img_gray, encoding="passthrough")
        self.img2occ_grid()
        self.publish_grid()
        # self.img2costmap()
        # self.publish_map()

    def img2costmap(self):
        """
        Converts image to costmap.

        """
        self.cost_map.header.stamp = rospy.Time.now()
        self.cost_map.header.frame_id = "costmap"
        self.cost_map.info.resolution = 1
        self.cost_map.info.width = self.width
        self.cost_map.info.height = self.height
        self.cost_map.info.origin.position.x = 0
        self.cost_map.info.origin.position.y = 0
        self.cost_map.info.origin.position.z = 0
        self.cost_map.info.origin.orientation.x = 0
        self.cost_map.info.origin.orientation.y = 0
        self.cost_map.info.origin.orientation.z = 0
        self.cost_map.info.origin.orientation.w = 1
        self.cost_map.data = self.img_gray.data

    def img2occ_grid(self):
        """
        Converts image to occupancy grid.

        """
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "map"
        self.grid.info.resolution = 1
        self.grid.info.width = self.width
        self.grid.info.height = self.height
        self.grid.info.origin.position.x = 0
        self.grid.info.origin.position.y = 0
        self.grid.info.origin.position.z = 0
        self.grid.info.origin.orientation.x = 0
        self.grid.info.origin.orientation.y = 0
        self.grid.info.origin.orientation.z = 0
        self.grid.info.origin.orientation.w = 1
        self.grid.data = self.img.data

    def publish_image(self):
        """
        Publishes the debug image.

        """
        print("Publishing occ_img")
        self.img.header.stamp = rospy.Time.now()
        self.img.header.frame_id = "occ_grid_img"
        self.img_pub.publish(self.img)

    def publish_grid(self):
        """
        Publishes the occupancy grid.

        """
        print("Publishing occ_grid")
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "grid"
        self.grid_pub.publish(self.grid)

    def publish_map(self):
        """
        Publishes the occupancy grid map and costmap.

        """
        print("Publishing map")
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "map"
        self.map_pub.publish(self.grid)
        self.cost_map.header.stamp = rospy.Time.now()
        self.cost_map.header.frame_id = "map"
        self.costmap_pub.publish(self.cost_map)
        
    def trig_grid(self, req):
        """
        Triggers the grid publishing.

        Args:
            req (TriggerRequest): Trigger request.

        Returns:
            bool: True if successful, False otherwise.
            str: Message indicating the result.

        """
        print("Triggering grid")
        self.publish_grid()
        return True, "Grid published"

    def trig_map(self, req):
        """
        Triggers the map publishing.

        Args:
            req (TriggerRequest): Trigger request.

        Returns:
            bool: True if successful, False otherwise.
            str: Message indicating the result.

        """
        print("Triggering map")
        self.publish_map()
        return True, "Map published"

def main(args):
    rospy.init_node('occ_map_node')
    occ_grid = OccGrid()
    # occ_grid.publish_grid()
    # occ_grid.publish_map()
    rospy.spin()
    return 0

if __name__ == '__main__':
    main(sys.argv)