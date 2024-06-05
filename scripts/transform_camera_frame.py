#!/usr/bin/env python
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import numpy as np

rospy.init_node('my_tf2_broadcaster')
br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "map"
t.child_frame_id = "pylon_camera"
t.transform.translation.x = 0.46
t.transform.translation.y = -0.2
t.transform.translation.z = 2
q = tf_conversions.transformations.quaternion_from_euler(np.pi, 0, 0)
t.transform.rotation.x = q[0]
t.transform.rotation.y = q[1]
t.transform.rotation.z = q[2]
t.transform.rotation.w = q[3]
br.sendTransform(t)
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    t.header.stamp = rospy.Time.now()
    br.sendTransform(t)
    rate.sleep()