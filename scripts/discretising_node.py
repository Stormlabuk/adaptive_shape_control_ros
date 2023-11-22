#!/usr/bin/env python
import rospy
from adaptive_ctrl.msg import rl_angles
from adaptive_ctrl.srv import DiscretiseCurve, DiscretiseCurveResponse, DiscretiseCurveRequest


def points_to_angles(req):
    print(req)
    return DiscretiseCurveResponse(req.px) #return the same request for now

def DiscretisingNode():
    rospy.init_node('discretising_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub = rospy.Publisher('rl_angles', rl_angles, queue_size=10)
    rospy.Service('discretise_curve', DiscretiseCurve, points_to_angles)
    print("Discretising node started")
    rospy.spin()


if __name__ == '__main__':
    try:
        DiscretisingNode()
    except rospy.ROSInterruptException:
        pass