#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from shapeforming_msgs.srv import GetInsertion, GetInsertionRequest
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import cv2
import numpy as np
import sys


class FindInserterNode:
    """
    Class representing a ROS node for finding an inserter in an image.

    Attributes:
        img_path (str): The path to the image file.
        bridge (CvBridge): The bridge object for converting between ROS Image messages and OpenCV images.
        img (numpy.ndarray): The grayscale image.
        pathSub_ (rospy.Subscriber): The subscriber for receiving the image file path.
        inserter_srv (rospy.Service): The service for finding the inserter.
        insertionPub (rospy.Publisher): The publisher for publishing the insertion point.
        phantomPub (rospy.Publisher): The publisher for publishing the image with the inserter removed.
        img_pub (rospy.Publisher): The publisher for publishing the original image.

    Methods:
        img_path_callback(self, msg): Callback function for the image file path subscriber.
        find_inserter(self, req): Method for finding the inserter and publishing the insertion point.
        pub_imgs(self, polygon): Method for publishing the images.
        deleteInserter(self, poly): Method for removing the inserter from the image.
        getPoly(self, img): Method for getting the polygon of the inserter.
        getInsertionPoint(self, poly): Method for calculating the insertion point of the inserter.
        constructPoint(self, poly): Method for constructing the insertion point coordinates.
        rotatePolygon(self, poly, angle, centroid): Method for rotating the polygon.
    """

    def __init__(self):
        rospy.init_node('find_inserter_node', anonymous=False)
        default_path = '/home/vittorio/ros_ws/src/adaptive_ctrl/inserter/4.inserter_p0.png'
        self.img_path = rospy.get_param('~img_path', default_path)
        self.bridge = CvBridge()
        self.img = cv2.imread(self.img_path, cv2.IMREAD_GRAYSCALE)
        if (self.img is None):
            sys.exit()
        _, self.img = cv2.threshold(self.img, 127, 255, cv2.THRESH_BINARY)
        self.pathSub_ = rospy.Subscriber(
            '/img_path', String, self.img_path_callback)
        self.inserter_srv = rospy.Service(
            '/find_inserter', GetInsertion, self.find_inserter)
        self.insertionPub = rospy.Publisher(
            '/insertion_point', Point, queue_size=1)
        self.phantomPub = rospy.Publisher('/phantom_img', Image, queue_size=1)
        self.img_pub = rospy.Publisher('/img', Image, queue_size=1)
        self.inserter_marker_pub = rospy.Publisher(
            '/ins_pos', Marker, queue_size=1)
        self.inserter_point_pub = rospy.Publisher(
            '/inserter_point', Point, queue_size=1)
        self.leftCircle = np.zeros((2, 1))
        self.rightCircle = np.zeros((2, 1))
        self.insertion_point = np.zeros((2, 1))

    def img_path_callback(self, msg):
        """
        Callback function for the image file path subscriber.

        Args:
            msg (std_msgs.msg.String): The message containing the file path.
        """
        file_path = msg.data
        self.img = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            sys.exit()
        _, self.img = cv2.threshold(self.img, 127, 255, cv2.THRESH_BINARY)
        req = GetInsertionRequest()
        rospy.wait_for_service('/find_inserter')
        inserter_pos = rospy.ServiceProxy('/find_inserter', GetInsertion)
        inserter_res = inserter_pos(req)
        inserter_point = np.array(
            [inserter_res.point.x, inserter_res.point.y, inserter_res.point.z])
        self.insertion_point = inserter_point
        self.inserter_point_pub.publish(inserter_res.point)

    def populate_marker(self, point):
        """
        Creates a marker object with the given point.

        Args:
            points (list): List of points in the form [x, y, z].

        Returns:
            Marker: The created marker object.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 0
        marker.scale.x = 10
        marker.scale.y = 10
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker_point = Point()
        marker_point.x = point[0]
        marker_point.y = point[1]
        marker_point.z = point[2] if len(point) > 2 else 0.0
        marker.points.append(marker_point)

        return marker

    def find_inserter(self, req):
        """
        Method for finding the inserter and publishing the insertion point.

        Args:
            req (adaptive_ctrl.srv.GetInsertionRequest): The request message.

        Returns:
            adaptive_ctrl.srv.GetInsertionResponse: The response message containing the insertion point.
        """
        img = np.array(self.img)
        img = self.img.copy()
        polygon = self.getPoly(img)
        insertion_point = self.getInsertionPoint(polygon)
        insertion_point_msg = Point()
        insertion_point_msg.x = insertion_point[0]
        insertion_point_msg.y = insertion_point[1]
        insertion_point_msg.z = 0.0
        self.insertionPub.publish(insertion_point_msg)
        self.pub_imgs(polygon)
        return True, insertion_point_msg

    def pub_imgs(self, polygon):
        """
        Method for publishing the images.

        Args:
            polygon (numpy.ndarray): The polygon of the inserter.
        """
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(
            self.img, encoding="passthrough"))
        self.deleteInserter(polygon)
        return

    def deleteInserter(self, poly):
        """
        Method for removing the inserter from the image.

        Args:
            poly (numpy.ndarray): The polygon of the inserter.
        """
        mask = np.zeros(self.img.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [poly], 255, cv2.LINE_8)
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        mask = cv2.dilate(mask, element)
        inserter_less_img = self.img.copy()
        inserter_less_img[mask == 255] = 0
        cv2.circle(inserter_less_img, np.int32(self.leftCircle), 15, 255, -1)
        cv2.circle(inserter_less_img, np.int32(self.rightCircle), 15, 255, -1)
        self.phantomPub.publish(self.bridge.cv2_to_imgmsg(
            inserter_less_img, encoding="passthrough"))
        self.inserter_marker_pub.publish(self.populate_marker(self.insertion_point))

    def getPoly(self, img):
        """
        Method for getting the polygon of the inserter.

        Args:
            img (numpy.ndarray): The grayscale image.

        Returns:
            numpy.ndarray: The polygon of the inserter.
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
        Method for calculating the insertion point of the inserter.

        Args:
            poly (numpy.ndarray): The polygon of the inserter.

        Returns:
            numpy.ndarray: The insertion point coordinates.
        """
        bounding_rect = cv2.minAreaRect(poly)
        centroid = bounding_rect[0]
        orientation = bounding_rect[2]
        rectPoints = np.intp(cv2.boxPoints(bounding_rect))
        p1, p2, p4 = rectPoints[0], rectPoints[1], rectPoints[3]
        a = np.linalg.norm(p1 - p2)
        b = np.linalg.norm(p1 - p4)
        left_hand_flag = a > b
        orientation = 90 - orientation if not left_hand_flag else -orientation
        rotatedPoly = self.rotatePolygon(poly, orientation, centroid)
        insertion_point = self.constructPoint(rotatedPoly)

        insertion_point = self.rotatePolygon(
            insertion_point, -orientation, centroid)
        insert_y = insertion_point[1]
        insert_x = insertion_point[0]
        width = 30
        self.leftCircle = np.array([insert_x - width, insert_y-5])
        self.rightCircle = np.array([insert_x + width, insert_y-5])
        # self.rotatePolygon(self.leftCircle, -orientation, centroid)
        # self.rotatePolygon(self.rightCircle, -orientation, centroid)
        return insertion_point + np.array([0,20])

    def constructPoint(self, poly):
        """
        Method for constructing the insertion point coordinates.

        Args:
            poly (numpy.ndarray): The polygon of the inserter.

        Returns:
            numpy.ndarray: The insertion point coordinates.
        """
        max_y = np.max(poly[:, 1])
        avg_x = np.mean(poly[:, 0])
        return np.array([avg_x, max_y])

    def rotatePolygon(self, poly, angle, centroid):
        """
        Method for rotating the polygon.

        Args:
            poly (numpy.ndarray): The polygon of the inserter.
            angle (float): The rotation angle in degrees.
            centroid (numpy.ndarray): The centroid of the polygon.

        Returns:
            numpy.ndarray: The rotated polygon.
        """
        angle_rad = np.deg2rad(angle)
        rotmatMan = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                              [np.sin(angle_rad), np.cos(angle_rad)]])
        rotatedPoly = np.dot(rotmatMan, (poly - centroid).T).T + centroid
        return rotatedPoly


def main():
    rospy.init_node('find_inserter_node')
    FindInserterNode()
    rospy.spin()


if __name__ == '__main__':
    main()
