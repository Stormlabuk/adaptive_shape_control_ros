#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from heuristic_planners.srv import GetPath, GetPathRequest, GetPathResponse

def main():
    rospy.init_node('service_trigger')

    # Trigger /srv_map service
    rospy.wait_for_service('/srv_map')
    map_service = rospy.ServiceProxy('/srv_map', Trigger)
    map_service()

    # Trigger /srv_grid service
    rospy.wait_for_service('/srv_grid')
    grid_service = rospy.ServiceProxy('/srv_grid', Trigger)
    grid_service()

    # Trigger /planner_ros_node/request_path service
    rospy.wait_for_service('/planner_ros_node/request_path')
    path_service = rospy.ServiceProxy('/planner_ros_node/request_path', GetPath)
    req = GetPathRequest()
    req.start.x = 433
    req.start.y = 87
    req.start.z = 0.0

    req.goal.x = 228.0
    req.goal.y = 249.0
    req.goal.z = 0.0
    
    # req.algorithm.data = "thetastar"
    path_service(req)

    # rospy.spin()

if __name__ == '__main__':
    main()