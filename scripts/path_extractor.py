#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from shapeforming_msgs.msg import tent_curvature
import numpy as np

class PubSubApp:
    def __init__(self):
        base_planner = "/planner_ros_node/"
        rospy.init_node('marker_to_path', anonymous=False)
        # rospy.Subscriber(base_planner + "path_points_markers_array", Marker, self.MarkerCB_)
        # rospy.Subscriber(base_planner + "path_line_markers_array", Marker, self.LineCB_)
        rospy.Subscriber(base_planner + "path", Path, self.PathCB_)
        self.pub = rospy.Publisher('tent_curvature_topic', tent_curvature, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz

    def PathCB_(self, msg):
        rospy.loginfo("PathCB_")
        # extract the x and y coordinates from the path
        x = []
        y = []
        for pose in msg.poses:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)
        # print x and y side by side
        print()
        print("x, len: ", len(x), ", y ", len(y))
        
        for i in range(len(x)):
            print(x[i], y[i])


def main():
    app = PubSubApp()
    rospy.spin()

if __name__ == '__main__':
    main()
