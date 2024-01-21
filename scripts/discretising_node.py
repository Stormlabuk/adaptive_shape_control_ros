#!/usr/bin/env python
import rospy
import numpy as np
from shapeforming_msgs.msg import rl_angles
from shapeforming_msgs.msg import tent_curvature
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveResponse, DiscretiseCurveRequest

class DiscretisingNode:
    def __init__(self):
        self.pub = rospy.Publisher('obv_angles', rl_angles, queue_size=10)
        rospy.init_node('discretising_node', anonymous=True)
        rospy.Service('discretise_curve', DiscretiseCurve, self.points_to_angles)
        rospy.Subscriber('tent_curvature_topic', tent_curvature, self.curve_callback)
        print("Discretising node started")
        rospy.spin()

    def curve_callback(self, data):
        num_points = data.num_points
        px = data.px
        py = data.py
        rospy.wait_for_service('discretise_curve')
        try:
            # print("Service found")
            discretise_curve = rospy.ServiceProxy(
                'discretise_curve', DiscretiseCurve)
            resp1 = discretise_curve(num_points, px, py)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        return

    def points_to_angles(self, req):
        x = np.array(req.px)
        y = np.array(req.py)
        # print(req)
        if(req.num_points > 0):
            xinter, yinter = self.disc_points(x, y, req.num_points)
            dx = xinter[+1:]-xinter[:-1]
            dy = yinter[+1:]-yinter[:-1]
            
            # Fixing the code
            vects = np.array([dx, dy]).T
            th = np.arctan2(vects[+1:, 0] * vects[:-1, 1] - vects[+1:, 1] * vects[:-1, 0], vects[+1:, 0] * vects[:-1, 0] + vects[+1:, 1] * vects[:-1, 1])
            angles = th * 180 / np.pi
            
            res = DiscretiseCurveResponse(angles)
            angles_msg = rl_angles(angles=res.angles, count=req.num_points)
            self.pub.publish(angles_msg)
            return res
        else:
            # print("Service called, but message empty. skipping")
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