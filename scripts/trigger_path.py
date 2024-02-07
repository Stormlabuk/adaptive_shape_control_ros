#!/usr/bin/env python
import rospy
from heuristic_planners.srv import GetPath, GetPathRequest, GetPathResponse
from shapeforming_msgs.srv import GetInsertion, GetInsertionResponse
from std_msgs.msg import String
import os


def main():
    rospy.init_node('service_trigger')

    # img_paths = os.listdir('/home/vittorio/ros_ws/src/adaptive_ctrl/inserter')
    # img_paths = [os.path.join('/home/vittorio/ros_ws/src/adaptive_ctrl/inserter', img_path) for img_path in img_paths]
    # # sort alphabetically
    # img_paths.sort()
    # print(img_paths)
    # img_paths = [
    #     '/home/vittorio/ros_ws/src/adaptive_ctrl/inserter/6.inserter_p30.png']

    # # img_paths = [os.path.join('/home/vittorio/ros_ws/src/adaptive_ctrl/inster', img_path) for img_path in img_paths]
    # img_path_pub = rospy.Publisher('/img_path', String, queue_size=1)
    # for img_path in img_paths:
    #     rospy.loginfo(f'Publishing image path: {img_path}')
    #     img_path_pub.publish(img_path)
    #     rospy.spin()

    rospy.wait_for_service('/find_inserter')
    inserter_pos = rospy.ServiceProxy('/find_inserter', GetInsertion)
    inserter_res = inserter_pos()
    start = inserter_res.point

    # Trigger /planner_ros_node/request_path service
    rospy.wait_for_service('/planner_ros_node/request_path')
    path_service = rospy.ServiceProxy(
        '/planner_ros_node/request_path', GetPath)
    req = GetPathRequest()
    # req.start.x = 433
    # req.start.y = 87
    # req.start.z = 0.0
    req.start = start

    req.goal.x = 308
    req.goal.y = 395
    req.goal.z = 0.0

    # req.algorithm.data = "thetastar"
    path_service(req)
    rospy.sleep(2)

    # rospy.spin()


if __name__ == '__main__':
    main()
