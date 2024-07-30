#!/usr/bin/env python
import rospy
import numpy as np
from shapeforming_msgs.msg import rl_angles
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveResponse
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf_conversions
import tf2_ros
import cv2

class DiscretisingNode:
    def __init__(self):
        rospy.init_node('discretising_node', anonymous=False)
        
        angle_pub_type = rospy.get_param('~angle_pub_type', 'des_')
        if angle_pub_type == 'obv_':
            self.marker_color = [1, 0.5, 0]  # orange
        elif angle_pub_type == 'des_':
            self.marker_color = [0.5, 0, 1]  # purple
        else:
            self.marker_color = [1, 0, 0]  # default to red
        # Pubs and subs
        self.obvAnglesPub_ = rospy.Publisher(angle_pub_type + 'angles', rl_angles, queue_size=10)
        self.marker_pub = rospy.Publisher(angle_pub_type + "viz_angles", Marker, queue_size=10)
        rospy.loginfo("Discretising node started. Name " + rospy.get_name() + " Type: " + angle_pub_type + "angles")

        # Services
        rospy.Service(angle_pub_type + 'discretise_curve', DiscretiseCurve, self.points_to_angles)
        print("Discretising node started")
        
        rospy.spin()

    def points_to_angles(self, req):
        num_points = req.tentacle.num_points
        x = np.array(req.tentacle.px)
        y = np.array(req.tentacle.py)
        if(num_points > 0):
            dx = np.diff(x)
            dy = np.diff(y)
            angles = np.arctan2(dy, dx) * 180 / np.pi
            
            res = DiscretiseCurveResponse(angles, True)
            angles_msg = rl_angles(angles=res.angles, count=num_points-1)
            self.obvAnglesPub_.publish(angles_msg)
            self.visualiseAngles(np.column_stack((x, y)), angles)
            return res
        else:
            self.clearMarkers()
            return DiscretiseCurveResponse([], False)


    def clearMarkers(self):
        """
        Clears all markers from RViz.
        """
        # rospy.loginfo("Clearing markers")
        marker = Marker()
        marker = self.populateMarker(marker, [], self.marker_color, 0)
        self.marker_pub.publish(marker)

    def populateMarker(self, marker, points, color, a):
        """
        Populates a Marker object with given points, color, and transparency.

        Args:
            marker (Marker): The Marker object to populate.
            points (list): List of points to add to the Marker.
            color (list): List of RGB values for the Marker color.
            a (float): Transparency value for the Marker.

        Returns:
            Marker: The populated Marker object.
        """
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.POINTS
        if(len(points) == 0):
            marker.action = Marker.DELETE
            return marker
        marker.action = Marker.ADD
        # marker.pose.orientation.x = 0
        # marker.pose.orientation.y = 0
        # marker.pose.orientation.z = 0
        # marker.pose.orientation.w = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        marker.scale.x = 10
        marker.scale.y = 10
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = a
        for point in points:
            marker_point = Point()
            marker_point.x = point[0]
            marker_point.y = point[1]
            marker_point.z = point[2] if len(point) > 2 else 0.0
            marker.points.append(marker_point)

        return marker

    def visualiseAngles(self, disc_points, angles):
        """
        Visualises the given angles in RViz.

        Args:
            angles (list): List of angles to visualise.
        """        
        start = disc_points[0]
        verification_points = []
        verification_points.append(start)
        dx, dy = np.diff(disc_points, axis=0).T
        distances = np.sqrt(dx**2 + dy**2)
        for i in range(1, len(angles)):
            angle = angles[i-1]
            distance = distances[i-1]
            dx = distance * np.cos(np.radians(angle))
            dy = distance * np.sin(np.radians(angle))
            point = verification_points[i-1] + [dx, dy]
            verification_points.append(point)
        verification_points.append(disc_points[-1])
        marker = Marker()
        marker = self.populateMarker(marker, verification_points, self.marker_color, 1)
        self.marker_pub.publish(marker)
        marker.header.frame_id = "map"
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        DiscretisingNode()
    except rospy.ROSInterruptException:
        pass