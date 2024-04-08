#!/usr/bin/env python
import rospy
import numpy as np
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveRequest
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class DiscretisePath:
    def __init__(self):
        rospy.init_node('discretise_path', anonymous=False)
        base_planner = "/planner_ros_node/"

        self.mm_pixel = rospy.get_param("~mm_pixel", 5) # 1mm = 5 pixel. Converts mm to pixel
        self.pixel_mm = 1 / self.mm_pixel
        rospy.Subscriber(base_planner + "path", Path, self.PathCB_)
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
        slice, link_no = self.getSlice(interpolated_path, 10)
        # rospy.loginfo("interpolated path length: %d, slice length: %d, link_no: %d" % (len(interpolated_path), len(slice), link_no))
        # 4. feed that to the discretise_curve service
        indices = np.linspace(0, len(slice) -
                              1, link_no, dtype=int) 
        disc_points = slice[indices]
        req = DiscretiseCurveRequest()
        req.tentacle.px = disc_points[:, 0].tolist()
        req.tentacle.py = disc_points[:, 1].tolist()
        req.tentacle.num_points = link_no
        discProxy = rospy.ServiceProxy('des_discretise_curve', DiscretiseCurve)
        res = discProxy(req)
        angles = res.angles
        success = res.success
        if not success:
            rospy.logerr("DiscretiseCurve service failed")
            return


    def getSlice(self, interpolated_path, link_l):
        fitted_links = 0
        sliced_points = []
        num_points = len(interpolated_path)
        link_l = int(link_l * self.mm_pixel)
        fitted_links = int( np.floor(num_points / link_l))
        if fitted_links > 5:
            fitted_links = 5
        elif fitted_links < 1:
            fitted_links = 1

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

    
if __name__ == '__main__':
    try:
        DiscretisePath()
    except rospy.ROSInterruptException:
        pass
