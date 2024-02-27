#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from shapeforming_msgs.srv import GetInsertion, GetInsertionRequest
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np


class FindInsertionPoint:
    def __init__(self) -> None:
        self.inserter_img = np.empty((0, 0), dtype=np.uint8)

        self.inserter_sub = rospy.Subscriber(
            "inserter_img", Image, self.inserter_callback)

        self.insertion_point_pub = rospy.Publisher(
            "insertion_point", Point, queue_size=10)
        self.insertion_point_marker = rospy.Publisher(
            "insertion_point_marker", Marker, queue_size=10)

        self.mm_pixel = rospy.get_param("~mm_pixel", 5) # 1mm = 5 pixel
        self.pixel_mm = 1 / self.mm_pixel

    def inserter_callback(self, msg):
        bridge = CvBridge()
        self.inserter_img = bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough")
        self.find_inserter_tip()
        return

    def find_inserter_tip(self):
        insertion_point = Point()

        contour, _ = cv2.findContours(
            self.inserter_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contour) > 0:
            ## select the largest contour
            contour = max(contour, key=cv2.contourArea)
            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)
            approx = approx.reshape(approx.shape[0], approx.shape[2])
        else:
            rospy.logerr("No contours found in the inserter image")
            return
        bounding_rect = cv2.minAreaRect(approx)
        centroid = bounding_rect[0]
        orientation = bounding_rect[2]
        rectPoints = np.intp(cv2.boxPoints(bounding_rect))
        p1, p2, p4 = rectPoints[0], rectPoints[1], rectPoints[3]
        a = np.linalg.norm(p1 - p2)
        b = np.linalg.norm(p1 - p4)
        left_hand_flag = a > b
        orientation = 90 - orientation if not left_hand_flag else -orientation
        polyRotated = self.rotatePolygon(
            approx, orientation, centroid)
        tipRotated = np.array([np.mean(polyRotated[:, 0]), np.max(polyRotated[:, 1])])
        scope_adj = np.array([1.2 * self.pixel_mm, -23.8 * self.pixel_mm])
        tipRotated = tipRotated + scope_adj
        ins_point = self.rotatePolygon(
            tipRotated, -orientation, centroid)
        insertion_point.x = ins_point[0]
        insertion_point.y = ins_point[1]
        self.insertion_point_pub.publish(insertion_point)
        marker = self.PopulateMarker([insertion_point.x, insertion_point.y, 0])
        self.insertion_point_marker.publish(marker)
        return

    def PopulateMarker(self, point):
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

if __name__ == "__main__":
    rospy.init_node('find_insertion_point', anonymous=False)
    FindInsertionPoint()
    rospy.spin()
