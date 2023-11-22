#!/usr/bin/env python
import rospy

import numpy as np
from adaptive_ctrl.srv import DiscretiseCurve, DiscretiseCurveResponse, DiscretiseCurveRequest


def sample_trigger():
    print("Waiting for service")
    px = np.array([1.0, 2.0, 3.0, 0.0, 0.0])
    py = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) 
    num_points = 5
    rospy.wait_for_service('discretise_curve')
    try:
        print("Service found")
        discretise_curve = rospy.ServiceProxy('discretise_curve', DiscretiseCurve)
        resp1 = discretise_curve(num_points, px, py)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    angles = sample_trigger()
    
