#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from shapeforming_msgs.msg import tent_cont
import numpy as np
from scipy import interpolate
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveRequest, DiscretiseCurveResponse
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import csv

class PathExtractor:
    def __init__(self):
        base_planner = "/planner_ros_node/"
        rospy.init_node('marker_to_path', anonymous=False)
        # rospy.Subscriber(base_planner + "path_points_markers_array", Marker, self.MarkerCB_)
        # rospy.Subscriber(base_planner + "path_line_markers_array", Marker, self.LineCB_)
        rospy.Subscriber(base_planner + "path", Path, self.PathCB_)      
        self.PathPub_ = rospy.Publisher('tent_cont_topic', tent_cont, queue_size=10)
        self.vizAnglePub_ = rospy.Publisher('viz_angles', Marker, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz


    def PathCB_(self, msg):
        # rospy.loginfo("PathCB_")
        # extract the x and y coordinates from the path
        x = []
        y = []
        for pose in msg.poses:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)
        x = np.array(x)
        y = np.array(y)
        path_points = np.column_stack((x, y))
        path_points = np.flip(path_points, axis=0)
        interpolated_path = self.interpolate_points(path_points)

        n = 6 # number of points to discretise the path
        indices = np.linspace(0, len(interpolated_path)-1, n, dtype=int)
        disc_points = interpolated_path[indices]
        dx, dy = np.diff(disc_points, axis=0).T
        distances = np.sqrt(dx**2 + dy**2)
        req = DiscretiseCurveRequest()
        req.tentacle.px = disc_points[:, 0].tolist()
        req.tentacle.py = disc_points[:, 1].tolist()
        req.tentacle.num_points = n
        discProxy = rospy.ServiceProxy('discretise_curve', DiscretiseCurve)
        res = discProxy(req)
        angles = res.angles
        # rospy.loginfo(f"Discretised angles: {angles}")

        # viz markers here
        start = interpolated_path[0]
        verification_points = []
        verification_points.append(start)
        for i in range(1, len(angles)):
            angle = angles[i-1]
            distance = distances[i-1]
            dx = distance * np.cos(np.radians(angle))
            dy = distance * np.sin(np.radians(angle))
            point = verification_points[i-1] + [dx, dy]
            verification_points.append(point)
        marker = self.create_marker(verification_points)
        self.vizAnglePub_.publish(marker)
        



    def create_marker(self, points):
        """
        Creates a marker object with the given points.

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
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        for point in points:
            # print(point)
            marker_point = Point()
            marker_point.x = point[0]
            marker_point.y = point[1]
            marker_point.z = point[2] if len(point) > 2 else 0.0
            marker.points.append(marker_point)

        return marker


    def interpolate_points(self, points):
        x = points[:, 0]
        y = points[:, 1]
        
        # Calculate the distances between consecutive points
        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        
        # Calculate the cumulative distances
        cumulative_distances = np.insert(np.cumsum(distances), 0, 0)
        
        # Calculate the total distance
        total_distance = cumulative_distances[-1]
        
        # Calculate the number of interpolated points
        num_interpolated_points = int(total_distance) + 1
        
        # Interpolate the x and y coordinates
        interpolated_x = np.interp(np.linspace(0, total_distance, num_interpolated_points), cumulative_distances, x)
        interpolated_y = np.interp(np.linspace(0, total_distance, num_interpolated_points), cumulative_distances, y)
        
        # Combine the interpolated x and y coordinates into a list of points
        interpolated_points = np.column_stack((interpolated_x, interpolated_y))
        
        return interpolated_points


def main():
    app = PathExtractor()
    rospy.spin()

if __name__ == '__main__':
    main()
