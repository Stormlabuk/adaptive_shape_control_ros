#!/usr/bin/env python
import rospy
import numpy as np
from shapeforming_msgs.msg import rl_angles
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveResponse
import cv2

class DiscretisingNode:
    def __init__(self):
        rospy.init_node('discretising_node', anonymous=False)
        
        angle_pub_type = rospy.get_param('~angle_pub_type', 'des_')

        # Pubs and subs
        self.obvAnglesPub_ = rospy.Publisher(angle_pub_type + 'angles', rl_angles, queue_size=10)
        rospy.loginfo("Discretising node started. Name " + rospy.get_name() + " Type: " + angle_pub_type + "angles")

        # Services
        rospy.Service('discretise_curve', DiscretiseCurve, self.points_to_angles)
        print("Discretising node started")
        
        rospy.spin()

    def points_to_angles(self, req):
        num_points = req.tentacle.num_points
        x = np.array(req.tentacle.px)
        y = np.array(req.tentacle.py)
        if(num_points > 0):
            dx = np.diff(x)
            dy = np.diff(y)
            angles = np.arctan2(dy, dx) * 180 / np.pi
            
            res = DiscretiseCurveResponse(angles, True)
            angles_msg = rl_angles(angles=res.angles, count=num_points)
            self.obvAnglesPub_.publish(angles_msg)
            return res
        else:
            return DiscretiseCurveResponse([], False)


if __name__ == '__main__':
    try:
        DiscretisingNode()
    except rospy.ROSInterruptException:
        pass