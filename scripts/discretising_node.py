#!/usr/bin/env python
import rospy
from adaptive_ctrl.msg import rl_angles
from adaptive_ctrl.srv import DiscretiseCurve, DiscretiseCurveResponse, DiscretiseCurveRequest

class DiscretisingNode:
    def __init__(self):
        self.pub = rospy.Publisher('rl_angles', rl_angles, queue_size=10)
        rospy.init_node('discretising_node', anonymous=True)
        rospy.Service('discretise_curve', DiscretiseCurve, self.points_to_angles)
        print("Discretising node started")
        rospy.spin()

    def points_to_angles(self, req):
        res = DiscretiseCurveResponse(req.px)
        angles_msg = rl_angles(angles=res.angles, count=req.num_points)
        self.pub.publish(angles_msg)
        return res

if __name__ == '__main__':
    try:
        DiscretisingNode()
    except rospy.ROSInterruptException:
        pass