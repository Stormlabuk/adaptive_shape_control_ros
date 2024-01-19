#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
import cv2
import sys, os

class OccGrid:

    def __init__(self):
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher('/img_grid', Image, queue_size=1)
        self.grid_pub = rospy.Publisher('/occ_grid', OccupancyGrid, queue_size=1)
        self.img_path = os.getcwd() + '/src/adaptive_ctrl/img/disp_cnt.png'
        self.img = cv2.imread(self.img_path, cv2.IMREAD_GRAYSCALE)
        # convert img to binary
        self.img[self.img > 0] = 100
        self.width = self.img.shape[0]
        self.height = self.img.shape[1]
        # self.img = cv2.bitwise_not(self.img)
        self.img = self.bridge.cv2_to_imgmsg(self.img, encoding="passthrough")
        self.grid = OccupancyGrid()
        self.img2occ_grid()

    def publish_image(self):
        print("Publishing occ_grid")
        self.img.header.stamp = rospy.Time.now()
        self.img.header.frame_id = "occ_grid"
        self.img_pub.publish(self.img)

    def img2occ_grid(self):
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "occ_grid"
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

    def publish_grid(self):
        print("Publishing occ_grid")
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "occ_grid"
        self.grid_pub.publish(self.grid)
        



def main(args):
    rospy.init_node('occ_grid', anonymous=True)
    occ_grid = OccGrid()
    rate = rospy.Rate(1)  # Set the rate to 1Hz

    while not rospy.is_shutdown():
        occ_grid.publish_grid()  # Publish the occ_grid image
        rate.sleep()  # Sleep to maintain the desired rate

    return 0

if __name__ == '__main__':
    main(sys.argv)