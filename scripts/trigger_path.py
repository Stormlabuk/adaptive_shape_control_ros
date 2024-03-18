#!/usr/bin/env python
import rospy
from heuristic_planners.srv import GetPath, GetPathRequest
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class TriggerPath:
    def __init__(self):
        rospy.init_node('service_trigger')
        self.inserter_sub = rospy.Subscriber('/insertion_point', Point, self.inserter_cb)
        self.goal_marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=10)
        self.goal_sub = rospy.Subscriber('/goal_point', Point, self.goal_cb)
        self.goal = Point()
        self.goal.x = 305
        self.goal.y = 102
        self.goal.z = 0

    def populateMarker(self, marker, points, color, a):
        """
        Populates a Marker object with given points, color, and transparency.

        Args:
            marker (Marker): The Marker object to populate.
            points (list): List of points to add to the Marker.
            color (list): List of RGB values for the Marker color.
            a (float): Transparency value for the Marker.

        Returns:
            Marker: The populated Marker object.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 0
        marker.scale.x = 10
        marker.scale.y = 10
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = a
        for point in points:
            marker_point = Point()
            marker_point.x = point.x
            marker_point.y = point.y
            marker_point.z = point.z
            marker.points.append(marker_point)

        return marker

    def inserter_cb(self, msg):
        rospy.wait_for_service('/planner_ros_node/request_path')
        path_trigger = rospy.ServiceProxy(
            '/planner_ros_node/request_path', GetPath)
        req = GetPathRequest()
        ins_point = msg
        req.start = ins_point
        goal = Point()
        goal.x = self.goal.x
        goal.y = self.goal.y
        goal.z = self.goal.z
        req.goal = goal
        # rospy.wait_for_message('/costmap', OccupancyGrid)
        res = path_trigger(req) 
        goal_marker = Marker()
        goal_marker = self.populateMarker(goal_marker, [self.goal], [1, 0, 0], 1)
        self.goal_marker_pub.publish(goal_marker)

    def goal_cb(self, msg):
        self.goal = msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    trigger_path = TriggerPath()
    trigger_path.run()
