#!/usr/bin/env python
import rospy
from heuristic_planners.srv import GetPath, GetPathRequest, GetPathResponse
from shapeforming_msgs.srv import GetInsertion, GetInsertionResponse
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import os



def inserter_cb(msg):
    rospy.wait_for_service('/planner_ros_node/request_path')
    path_trigger = rospy.ServiceProxy(
        '/planner_ros_node/request_path', GetPath)
    req = GetPathRequest()
    ins_point = msg
    req.start = ins_point
    goal = Point()
    goal.x = 308
    goal.y = 395
    goal.z = 0
    req.goal = goal
    res = path_trigger(req) 
    
    return

def main():
    rospy.init_node('service_trigger')
    inserter_sub = rospy.Subscriber('/inserter_pos', Point, inserter_cb)
    rospy.spin()    



if __name__ == '__main__':
    main()
