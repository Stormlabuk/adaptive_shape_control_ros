#!/usr/bin/env python
import rospy
import numpy as np
from shapeforming_msgs.msg import tent_continuous
from shapeforming_msgs.msg import rl_angles
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveRequest, DiscretiseCurveResponse
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class DiscretisePath:
    def __init__(self):
        rospy.init_node('discretise_path', anonymous=False)
        base_planner = "/planner_ros_node/"

        self.rl_num = rospy.get_param('~rl_num', 6)
        self.mm_pixel = rospy.get_param("~mm_pixel", 5) # 1mm = 5 pixel
        self.pixel_mm = 1 / self.mm_pixel
        rospy.Subscriber(base_planner + "path", Path, self.PathCB_)
        self.marker_pub = rospy.Publisher("viz_angles", Marker, queue_size=10)
        rospy.spin()

    def PathCB_(self, msg):
        # 1. get x,y coordinates out of the msg
        x, y = [], []
        for pose in msg.poses:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)
        x = np.array(x)
        y = np.array(y)
        # 2. interpolate to a continuous path
        path_points = np.flip(np.column_stack((x, y)), axis=0)
        interpolated_path = self.interpolate_points(path_points)
        # 3. find and slice the first 50mm worth of points
        slice, link_no = self.getSlice(interpolated_path, 50)
        rospy.loginfo("interpolated path length: %d, slice length: %d, link_no: %d" % (len(interpolated_path), len(slice), link_no))
        # 4. feed that to the discretise_curve service
        indices = np.linspace(0, len(slice) -
                              1, link_no, dtype=int)  # indices of rl_num evenly spaced points
        disc_points = slice[indices]
        req = DiscretiseCurveRequest()
        req.tentacle.px = disc_points[:, 0].tolist()
        req.tentacle.py = disc_points[:, 1].tolist()
        req.tentacle.num_points = link_no+1
        discProxy = rospy.ServiceProxy('discretise_curve', DiscretiseCurve)
        res = discProxy(req)
        angles = res.angles
        success = res.success
        # 6. visualise the angles in rviz
        start = slice[0]
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
        marker = Marker()
        marker = self.populateMarker(marker, verification_points, [0, 1, 0], 1)
        self.marker_pub.publish(marker)


    def getSlice(self, interpolated_path, link_l):
        # # Convert K from mm to pixels
        # K_pixels = K_mm * (self.mm_2_pixel)

        # # Calculate the distances between consecutive points
        # distances = np.sqrt(
        #     np.sum(np.diff(interpolated_path, axis=0)**2, axis=1))

        # # Calculate the cumulative distances along the path
        # cumulative_distances = np.insert(np.cumsum(distances), 0, 0)

        # # Find the index where the cumulative distance exceeds K_pixels
        # idx = np.searchsorted(cumulative_distances, K_pixels)
        # fitted_links = 5
        # if idx >= len(interpolated_path):
        #     idx = len(interpolated_path) - 1
        # else:
        #     fitted_links = int(np.floor(idx / self.mm_2_pixel))
        #     idx = fitted_links * self.mm_2_pixel
        # # If the cumulative distance at idx is exactly K_pixels, use all points up to idx
        # if cumulative_distances[idx] == K_pixels:
        #     return interpolated_path[:idx+1]

        # # Otherwise, interpolate the last point
        # last_point = interpolated_path[idx-1] + (K_pixels - cumulative_distances[idx-1]) / \
        #     distances[idx-1] * (interpolated_path[idx] -
        #                         interpolated_path[idx-1])
        # return np.vstack((interpolated_path[:idx], last_point)), fitted_links
        fitted_links = 0
        sliced_points = []
        num_points = len(interpolated_path)
        fitted_links = int( np.floor(num_points / link_l))
        if fitted_links > 5:
            fitted_links = 5
            sliced_points = interpolated_path[:5]

        indices_spanned = fitted_links * link_l
        if indices_spanned < num_points:
            sliced_points = interpolated_path[:indices_spanned]
        

        else:
            sliced_points = interpolated_path
        return sliced_points, fitted_links

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
        interpolated_x = np.interp(np.linspace(
            0, total_distance, num_interpolated_points), cumulative_distances, x)
        interpolated_y = np.interp(np.linspace(
            0, total_distance, num_interpolated_points), cumulative_distances, y)

        # Combine the interpolated x and y coordinates into a list of points
        interpolated_points = np.column_stack((interpolated_x, interpolated_y))

        return interpolated_points

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


if __name__ == '__main__':
    try:
        DiscretisePath()
    except rospy.ROSInterruptException:
        pass
