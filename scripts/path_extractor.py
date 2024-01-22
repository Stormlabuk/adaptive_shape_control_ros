#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from shapeforming_msgs.msg import tent_cont
import numpy as np
from scipy import interpolate
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveRequest, DiscretiseCurveResponse
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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
        rospy.loginfo("PathCB_")
        # extract the x and y coordinates from the path
        x = []
        y = []
        for pose in msg.poses:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)

        points = self.evenly_spaced_points(x, y, 300)
        tent_cont_msg = tent_cont()
        tent_cont_msg.num_points = 6
        # print(points.shape)
        tent_cont_msg.px = points[:, 0].tolist()
        tent_cont_msg.py = points[:, 1].tolist()
        # self.PathPub_.publish(tent_cont_msg)

        rospy.wait_for_service('discretise_curve')
        disc_curve = rospy.ServiceProxy('discretise_curve', DiscretiseCurve)
        req = DiscretiseCurveRequest()
        req.tentacle = tent_cont_msg
        angles = disc_curve(req)
        angles_np = np.array(angles.angles)
        # angles_np = np.flip(angles_np)
        l = 50
        dx = np.cos(angles_np*np.pi/180)*l
        dy = np.sin(angles_np*np.pi/180)*l
        rl_points = np.zeros((len(angles_np), 2))
        rl_points[0] = points[-1]
        for i in range(1, len(angles_np)):
            rl_points[i] = rl_points[i-1] + [dx[i-1] *-1, dy[i-1] ]

        # print(rl_points.shape)
        # print(rl_points)
        # print(angles_np)
        # # Publish the points as a marker
        marker = self.create_marker(rl_points)
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


    def evenly_spaced_points(self, x, y, n):
        """
        Generates evenly spaced points along a curve.

        Args:
            x (array-like): The x-coordinates of the original points.
            y (array-like): The y-coordinates of the original points.
            n (int): The number of evenly spaced points to generate.

        Returns:
            array-like: An array of x-y coordinate pairs representing the evenly spaced points.
        """
        # Create a function which represents the old relation of y with respect to x
        f = interpolate.interp1d(x, y)

        # Create new evenly spaced x values
        new_x = np.linspace(min(x), max(x), n)

        # Use the function to find the new y values
        new_y = f(new_x)

        # Merge the new x, y values and transpose it to get x-y coordinate pairs
        new_points = np.column_stack((new_x,new_y))

        return new_points


def main():
    app = PathExtractor()
    rospy.spin()

if __name__ == '__main__':
    main()
