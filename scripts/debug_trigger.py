#!/usr/bin/env python
import rospy
from shapeforming_msgs.msg import rl_angles
from shapeforming_msgs.srv import CalcInitialField




if __name__ == '__main__':
    rospy.init_node('debug_trigger', anonymous=False)
    
    # test deformation
    pub = rospy.Publisher('/des_angles', rl_angles, queue_size=1)
    rate = rospy.Rate(1) # 1hz
    msg = rl_angles()
    msg.angles = [10.0, 10.0, 10.0, 10.0, 10.0]
    msg.count = 5

    rospy.sleep(1)
    pub.publish(msg)

    rospy.wait_for_service("precomputation/calc_initial_field")
    calc_initial_field = rospy.ServiceProxy("precomputation/calc_initial_field", CalcInitialField)
    calc_initial_field()