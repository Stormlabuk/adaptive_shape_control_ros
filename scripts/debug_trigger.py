#!/usr/bin/env python
import rospy
from shapeforming_msgs.msg import rl_angles
from geometry_msgs.msg import Vector3
from shapeforming_msgs.srv import CalcInitialField, CalcInitialFieldResponse, CalcInitialFieldRequest



if __name__ == '__main__':
    rospy.init_node('debug_trigger', anonymous=False)
    
    # test deformation
    pub = rospy.Publisher('/des_angles', rl_angles, queue_size=1)
    rate = rospy.Rate(1) # 1hz
    tent = rl_angles()
    tent.angles = [10.0, 10.0, 10.0, 10.0, 10.0]
    tent.count = 5
    req = CalcInitialFieldRequest()
    req.tentacle = tent
    req.orientation = Vector3(0, 90, 0)

    rospy.wait_for_service("precomputation/calc_initial_field")
    calc_initial_field = rospy.ServiceProxy("precomputation/calc_initial_field", CalcInitialField)
    field_res = CalcInitialFieldResponse()
    field_res = calc_initial_field(req)
    print(field_res)
