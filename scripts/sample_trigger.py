#!/usr/bin/env python
import rospy
import csv
import numpy as np
import os.path
from adaptive_ctrl.srv import DiscretiseCurve, DiscretiseCurveResponse, DiscretiseCurveRequest


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
    if(len(x) != len(y)):
        print("Error: x and y are not the same length")
    else:
        return x, y, len(x)

def sample_trigger():
    print("Waiting for service")
    
    px, py, num_points = read_csv()
    
    rospy.wait_for_service('discretise_curve')
    try:
        print("Service found")
        discretise_curve = rospy.ServiceProxy('discretise_curve', DiscretiseCurve)
        resp1 = discretise_curve(num_points, px, py)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    angles = sample_trigger()
    
