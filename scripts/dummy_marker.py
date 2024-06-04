#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def publish_marker(x, y):
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0  # Fully opaque
    marker.color.r = 0.0  # Green
    marker.color.g = 1.0
    marker.color.b = 0.0

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 3:
        print("Usage: rosrun adaptive_ctrl dummy_marker x y")
        sys.exit(1)
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    try:
        publish_marker(x, y)
    except rospy.ROSInterruptException:
        pass