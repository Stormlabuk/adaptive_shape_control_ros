#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
import cv2
import numpy as np
import sys


class FindInserterNode:
    """
    Class representing a ROS node for finding the inserter in an image.

    Attributes:
    - img_path (str): The path to the image file.
    - bridge (CvBridge): The OpenCV bridge for converting images between ROS and OpenCV formats.
    - img (numpy.ndarray): The grayscale image.
    - inserter_srv (rospy.Service): The ROS service for finding the inserter.
    - insertionPub (rospy.Publisher): The ROS publisher for publishing the insertion point.
    - phantomPub (rospy.Publisher): The ROS publisher for publishing the phantom image.
    - img_pub (rospy.Publisher): The ROS publisher for publishing the image.
    - img_to_pub (sensor_msgs.msg.Image): The image message to be published.
    """

    def __init__(self):
        rospy.init_node('find_inserter_node', anonymous=False)
        default_path = '/home/vittorio/ros_ws/src/adaptive_ctrl/img/disp_cnt_inserter_pos2.png'
        self.img_path = rospy.get_param('~img_path', default_path)
        self.bridge = CvBridge()
        self.img = cv2.imread(self.img_path, cv2.IMREAD_GRAYSCALE)
        if (self.img is None):
            print("Image not found")
            sys.exit()
        _, self.img = cv2.threshold(self.img, 127, 255, cv2.THRESH_BINARY)
        self.inserter_srv = rospy.Service('/find_inserter', Trigger, self.find_inserter)
        self.insertionPub = rospy.Publisher('/insertion_point', Point, queue_size=1)
        self.phantomPub = rospy.Publisher('/phantom_img', Image, queue_size=1)
        self.img_pub = rospy.Publisher('/img', Image, queue_size=1)
        # self.img_to_pub = self.bridge.cv2_to_imgmsg(self.img, encoding="passthrough")
        # self.img_pub.publish(self.img_to_pub)

    def find_inserter(self, req):
        """
        Callback function for the find_inserter service.

        Parameters:
        - req: The service request.

        Returns:
        - success: A boolean indicating whether the insertion point was found.
        - message: A string message indicating the result of the service call.
        """
        img = np.array(self.img)
        img = img.astype(self.img.copy())
        polygon = self.getPoly(img)
        insertion_point = self.getInsertionPoint(polygon)
        insertion_point_msg = Point()
        insertion_point_msg.x = insertion_point[0]
        insertion_point_msg.y = insertion_point[1]
        insertion_point_msg.z = 0.0
        self.insertionPub.publish(insertion_point_msg)
        self.pub_imgs()
        return True, "Found insertion point"

    def pub_imgs(self):
        self.deleteInserter(self.getPoly(self.img))
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, encoding="passthrough"))
        return

    def deleteInserter(self, poly):
        """
        Deletes the inserter from the image.

        Parameters:
        - poly: The polygon representing the inserter.

        Returns:
        None
        """
        cv2.fillPoly(self.img, [poly], 0)
        self.img = self.bridge.cv2_to_imgmsg(self.img, encoding="passthrough")
        self.phantomPub.publish(self.img)

    def getPoly(self, img):
        """
        Finds and approximates the polygon shape of the smallest contour 
        in the given contour img.

        Parameters:
        - img: The input image. Must be a binary image of contours.

        Returns:
        - approx: The approximated polygon shape of the image.
        """
        contours, _ = cv2.findContours(
            img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        min_contour = min(contours, key=cv2.contourArea)
        approx = cv2.approxPolyDP( 
                min_contour, 0.01 * cv2.arcLength(min_contour, True), True) 
        approx = approx.reshape(approx.shape[0], approx.shape[2])
        return approx

    def getInsertionPoint(self, poly):
        """
        Calculates the insertion point of a polygon.

        Parameters:
        - poly: The polygon represented as a list of points.

        Returns:
        - insertion_point: The coordinates of the insertion point as a tuple.
        """
        bounding_rect = cv2.minAreaRect(poly)
        orientation = bounding_rect[2]
        rotated_poly = self.rotatePolygon(poly, orientation)
        insertion_point = self.constructPoint(rotated_poly)
        center = np.mean(poly, axis=0)
        insertion_point = self.rotate_point(insertion_point, center, -orientation)
        return insertion_point

    def constructPoint(self, poly):
        """
        Constructs a point based on the given polygon. 
        The point will have the maximum possible y value that is convex to the set and the average x value available.

        Parameters:
        - poly: The polygon represented as a numpy array.

        Returns:
        - point: The constructed point as a numpy array.
        """
        max_y = np.max(poly[:, 1])
        avg_x = np.mean(poly[:, 0])
        return np.array([avg_x, max_y])

    def rotatePolygon(self, pts, angle):
        """
        Rotate the polygon described by the list of points `pts` by angle `angle`.

        Parameters:
        - pts: The points of the polygon as a numpy array.
        - angle: The angle by which to rotate the polygon, assumed in degrees.

        Returns:
        - rotated_pts: The rotated polygon as a numpy array.
        """
        center_x = np.mean(pts[:, 0])
        center_y = np.mean(pts[:, 1])
        angle_rad = -angle / 180.0 * np.pi
        rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                    [np.sin(angle_rad), np.cos(angle_rad)]])
        pts_shifted = pts - np.array([center_x, center_y])
        rotated_pts = np.dot(pts_shifted, rotation_matrix.T) + np.array([center_x, center_y])
        return rotated_pts

    def rotate_point(self, pt, center, angle):
        """
        Rotate the point `pt` around `center` by `angle` degrees.

        Parameters:
        - pt: The point to rotate as a numpy array.
        - center: The center of rotation as a numpy array.
        - angle: The angle by which to rotate the point.

        Returns:
        - rotated_pt: The rotated point as a numpy array.
        """
        angle_rad = -angle / 180.0 * np.pi
        rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                    [np.sin(angle_rad), np.cos(angle_rad)]])
        pt_shifted = pt - center
        rotated_pt = np.dot(pt_shifted, rotation_matrix.T) + center
        return rotated_pt

def main():
    # Initialize the ROS node
    rospy.init_node('find_inserter_node')
    FindInserterNode()
    # Add your code here

    # Spin the node
    rospy.spin()


if __name__ == '__main__':
    main()
