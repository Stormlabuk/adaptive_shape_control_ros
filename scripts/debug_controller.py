#!/usr/bin/env python
import rospy
from shapeforming_msgs.msg import rl_angles
from ros_coils.msg import magField
from geometry_msgs.msg import Vector3

if __name__ == '__main__':
    rospy.init_node('debug_controller', anonymous=False)

    des_anglePub = rospy.Publisher('/des_angles', rl_angles, queue_size=1)
    obv_anglePub = rospy.Publisher('/obv_angles', rl_angles, queue_size=1)
    base_fieldPub = rospy.Publisher('/base_field', magField, queue_size=1)
    rate = rospy.Rate(0.5) # 1hz
    des = rl_angles()
    des.angles = [10.0, 10.0, 10.0, 10.0, 10.0]
    des.count = 5
    obv = rl_angles()
    obv.angles = [0, 0, 0, 0, 0]
    obv.count = 5
    base_field = magField()
    base_field.bx = 1
    base_field.by = 0
    base_field.bz = 0

    while not rospy.is_shutdown():
        des_anglePub.publish(des)
        obv_anglePub.publish(obv)
        base_fieldPub.publish(base_field)
        rate.sleep()

