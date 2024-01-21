#!/usr/bin/env python
import rospy
import numpy as np
from shapeforming_msgs.msg import rl_angles
from shapeforming_msgs.msg import tent_cont
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveResponse

class DiscretisingNode:
    def __init__(self):
        rospy.init_node('discretising_node', anonymous=False)
        
        angle_pub_type = rospy.get_param('~angle_pub_type', 'des_')

        # Pubs and subs
        self.obvAnglesPub_ = rospy.Publisher(angle_pub_type + 'angles', rl_angles, queue_size=10)
        
        # Services
        rospy.Service('discretise_curve', DiscretiseCurve, self.points_to_angles)
        print("Discretising node started")
        
        rospy.spin()

    def points_to_angles(self, req):
        num_points = req.tentacle.num_points
        x = np.array(req.tentacle.px)
        y = np.array(req.tentacle.py)
        # print(req)
        if(num_points > 0):
            xinter, yinter = self.disc_points(x, y, num_points)
            dx = xinter[+1:]-xinter[:-1]
            dy = yinter[+1:]-yinter[:-1]
            
            vects = np.array([dx, dy]).T
            th = np.arctan2(vects[+1:, 0] * vects[:-1, 1] - vects[+1:, 1] * vects[:-1, 0], vects[+1:, 0] * vects[:-1, 0] + vects[+1:, 1] * vects[:-1, 1])
            angles = th * 180 / np.pi
            
            res = DiscretiseCurveResponse(angles)
            angles_msg = rl_angles(angles=res.angles, count=num_points)
            self.obvAnglesPub_.publish(angles_msg)
            return res
        else:
            return DiscretiseCurveResponse([])

    def disc_points(self, x, y, num_points = 6):
        dx, dy = x[+1:]-x[:-1],  y[+1:]-y[:-1]
        ds = np.array((0, *np.sqrt(dx*dx+dy*dy)))
        s = np.cumsum(ds)
        xinter = np.interp(np.linspace(0, s[-1], num_points), s, x)
        yinter = np.interp(np.linspace(0, s[-1], num_points), s, y)
        xinter = np.insert(xinter, 0, xinter[0])
        yinter = np.insert(yinter, 0, 0)
        
        # Invert xinter and yinter
        xinter = np.flip(xinter)
        yinter = np.flip(yinter)
        return xinter, yinter

if __name__ == '__main__':
    try:
        DiscretisingNode()
    except rospy.ROSInterruptException:
        pass