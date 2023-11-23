import rospy
from adaptive_ctrl.msg import tent_curvature
from adaptive_ctrl.srv import DiscretiseCurve

class SensingDiscBridge:
    def __init__(self):
        rospy.init_node('sensing_disc_bridge', anonymous=True)
        rospy.Subscriber('tent_curvature_topic', tent_curvature, self.callback)

    def callback(self, data):
        num_points = data.num_points
        px = data.px
        py = data.py
        rospy.wait_for_service('discretise_curve')
        try:
            print("Service found")
            discretise_curve = rospy.ServiceProxy(
                'discretise_curve', DiscretiseCurve)
            resp1 = discretise_curve(num_points, px, py)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    bridge = SensingDiscBridge()
    bridge.run()
