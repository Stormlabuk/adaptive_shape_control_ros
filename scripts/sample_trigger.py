#!/usr/bin/env python
import rospy
import csv
import numpy as np
import os.path
from shapeforming_msgs.srv import DiscretiseCurve, DiscretiseCurveResponse, DiscretiseCurveRequest


def read_csv():
    filename = os.path.dirname(__file__) + '/../csv/extractedPoints.csv'
    with open(filename, newline='') as csvfile:
        data = list(csv.reader(csvfile))
        skipped_data = data[2:]  # Skip the first two lines
        N = 348  # Specify the number of rows to read
        selected_data = skipped_data[:N]  # Read the next N rows
    data = np.array(selected_data)
    x = data[:, 0].astype(float)
    y = data[:, 1].astype(float)
    if (len(x) != len(y)):
        print("Error: x and y are not the same length")
    else:
        return x, y


def sample_trigger():
    print("Waiting for service")
    num_points = 6  # desired number of joints.
    # angles will be calculated for num_points-1
    # this should be a ros_param soon
    px, py = read_csv()

    rospy.wait_for_service('discretise_curve')
    try:
        print("Service found")
        discretise_curve = rospy.ServiceProxy(
            'discretise_curve', DiscretiseCurve)
        resp1 = discretise_curve(num_points, px, py)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    angles = sample_trigger()
