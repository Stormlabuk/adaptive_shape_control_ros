#!/usr/bin/env python
import rospy
from shapeforming_msgs.msg import rl_angles
from geometry_msgs.msg import Vector3
from shapeforming_msgs.srv import CalcInitialField, CalcInitialFieldResponse, CalcInitialFieldRequest

def des_cb(msg):
    req = CalcInitialFieldRequest()
    req.tentacle = msg
    req.orientation = Vector3(0, 90, 0)

    rospy.wait_for_service("precomputation/calc_initial_field")
    calc_initial_field = rospy.ServiceProxy("precomputation/calc_initial_field", CalcInitialField)
    field_res = CalcInitialFieldResponse()
    field_res = calc_initial_field(req)
    print(field_res)


if __name__ == '__main__':
    rospy.init_node('debug_trigger', anonymous=False)
    rospy.loginfo("Debug Trigger Node Started")
    # test deformation
    rospy.wait_for_service("precomputation/calc_initial_field")
    msg = rospy.wait_for_message("obv_angles", rl_angles)
    des_cb(msg) 