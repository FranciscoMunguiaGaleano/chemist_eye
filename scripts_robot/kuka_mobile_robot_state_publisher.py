#!/usr/bin/env python
import rospy
import tf
import math
import heapq
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import time


NODES_POSITIONS = {
    1: [-2, 8.3], 2: [-1, 11], 22: [-12.4, 6.5], 3: [-1, 12], 4: [-1.5, 10.5], 24: [-3, 2.5],
    5: [-1.5, 12.5], 25: [-3, 3], 6: [-1.5, 8], 26: [-3.4, 2.5], 7: [-0.7, 8], 27: [-3.4, 2],
    8: [-1.8, 12.8], 28: [-4, 2.5], 9: [-2.8, 8.3], 29: [-2, 2.5], 10: [-4, 8.3], 30: [-1.8, 3],
    11: [-5, 8.3], 31: [-8.8, 11], 12: [-5, 3.7], 32: [-12.5, 9.5], 13: [-5.2, 3.5], 33: [-12.5, 8],
    34: [-1.8, 2], 15: [-8.8, 4.5], 35: [-12.5, 12.5], 16: [-8.8, 7], 36: [-12.5, 14.5],
    17: [-10.1, 7], 37: [-12.5, 10], 18: [-12.2, 7], 38: [-6.6, 3.3], 19: [-12.5, 6], 39: [-13.5, 7.5],
    20: [-12.5, 5.5]
}

NODES_RELATIONSHIPS = {
    1: {9: [90, 90], 6: [0, 0]},
    2: {4: [0, 90], 3: [0, 0]},
    3: {2: [0, 0], 5: [90, 90]},
    4: {1: [0, 0], 2: [0, 0]},
    5: {3: [0, 0], 8: [90, 90]},
    6: {1: [0, 0], 7: [0, 0]},
    7: {6: [0, 0]},
    8: {5: [90, 90]},
    9: {10: [0, 0], 1: [0, 0]},
    10: {9: [0, 0], 11: [0, 0]},
    11: {10: [0, 0], 12: [0, 0]},
    12: {11: [0, 0], 13: [0, 90]},
    13: {12: [0, 0], 38: [90, 0], 28: [90, 90]},
    15: {16: [0, 0], 38: [0, 0]},
    16: {15: [0, 0], 17: [0, 0], 31: [0, 0]},
    17: {18: [0, 0], 16: [0, 0]},
    18: {17: [0, 0], 22: [0, 0], 33: [0, 0]},
    19:{20:[90,90],22:[0,0],33:[0,0]},
    20:{19:[90,0]},
    22:{18:[0,0],19:[0,0]},
    24:{25:[90,90],26:[90,90],29:[90,90]},
    25:{24:[90,90]},
    26:{24:[90,90],27:[90,90], 28:[90,90]},
    27:{26:[90,90]},
    28:{13:[90,90], 26:[90,90]},
    29:{24:[90,90],30:[90,90],34:[90,90]},
    30:{29:[30,30]},
    31:{16:[0,0]},
    32:{33:[0,0],37:[0,0]},
    33:{18:[0,0],32:[0,0],39:[0,0]},
    34:{29:[90,90]},
    35:{36:[0,90],37:[0,0]},
    36:{35:[90,0]},
    37:{32:[0,0],35:[0,0]},
    38:{13:[0,0],15:[0,0]},
    39:{18:[0,0]},
}

COLOR_MAP = {
    0: (1.0, 0.0, 0.0, 0.8),  # Red
    1: (0.0, 1.0, 0.0, 0.8),  # Green
    2: (1.0, 1.0, 0.0, 0.8),  # Yellow
    3: (0.0, 0.0, 1.0, 0.8)   # Blue
}

PATH_COLOR = (1.0, 0.0, 1.0, 1.0)  # Pink

class RobotNavigator:
    def __init__(self):
        rospy.init_node('mobile_base_tf_broadcaster')
        self.ns = rospy.get_param('~ns_name', "robot1")
        self.path_pub = rospy.Publisher(self.ns + '_visualization_path_marker_array', MarkerArray, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)
        self.current_yaw = math.radians(90)

    def find_shortest_path(self, start, goal):
        queue = [(0, start, [])]
        visited = set()
        while queue:
            cost, node, path = heapq.heappop(queue)
            if node in visited:
                continue
            path = path + [node]
            if node == goal:
                return path
            visited.add(node)
            for neighbor in NODES_RELATIONSHIPS.get(node, {}):
                heapq.heappush(queue, (cost + 1, neighbor, path))
        return []

    def draw_path(self, path, marker_array):
        path_marker = Marker()
        path_marker.header.frame_id = "world"
        path_marker.ns = self.ns + "path"
        path_marker.id = 2000
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.06
        path_marker.color.r, path_marker.color.g, path_marker.color.b, path_marker.color.a = PATH_COLOR
        for node in path:
            point = Point()
            point.x, point.y = NODES_POSITIONS[node]
            point.z = 0.1
            path_marker.points.append(point)
        marker_array.markers.append(path_marker)

    def obstacles_callback(self, msg):
        return

    def navigate_robot(self, start_node, goal_node):
        return

    def move_base(self):
        start_node = rospy.get_param('~start_node', 24)
        goal_node = rospy.get_param('~goal_node', 36)
        automatic_routine = rospy.get_param('~automatic', False)

        path = self.find_shortest_path(start_node, goal_node)
        marker_array = MarkerArray()
        self.draw_path(path, marker_array)
        self.path_pub.publish(marker_array)

        while not rospy.is_shutdown():
            marker_array = MarkerArray()
            if len(path) > 0 and automatic_routine:
                self.draw_path(path, marker_array)
                self.path_pub.publish(marker_array)
                self.follow_path(path)
            else:
                x = NODES_POSITIONS[start_node][0]
                y = NODES_POSITIONS[start_node][1]
                yaw = math.radians(90)
                self.br.sendTransform((x, y, 0),
                                     tf.transformations.quaternion_from_euler(0, 0, yaw),
                                     rospy.Time.now(),
                                     self.ns + "_kuka_mobile_base_link",
                                     "world")
            self.rate.sleep()

    def follow_path(self, path):
        try:
            for i in range(0, len(path) - 1):
                self.move_to_node(path[i], path[i + 1])
            for i in range(len(path) - 1, 0, -1):
                self.move_to_node(path[i], path[i - 1])
        except Exception as e:
            rospy.loginfo(f'[Error] {e}')

    def move_to_node(self, node_i, node_i_):
        x_dist = (NODES_POSITIONS[node_i_][0] - NODES_POSITIONS[node_i][0]) / 100
        y_dist = (NODES_POSITIONS[node_i_][1] - NODES_POSITIONS[node_i][1]) / 100
        yaw_dist_at_node_i = (NODES_RELATIONSHIPS[node_i][node_i_][0] - math.degrees(self.current_yaw)) / 100

        if yaw_dist_at_node_i != 0:
            x = NODES_POSITIONS[node_i][0]
            y = NODES_POSITIONS[node_i][1]
            for i in range(0, 101):
                yaw_i = self.current_yaw + math.radians(yaw_dist_at_node_i) * i
                self.br.sendTransform((x, y, 0),
                                    tf.transformations.quaternion_from_euler(0, 0, yaw_i - math.radians(90)),
                                    rospy.Time.now(),
                                    self.ns + "_kuka_mobile_base_link",
                                    "world")
                time.sleep(0.01)
            self.current_yaw = math.radians(NODES_RELATIONSHIPS[node_i][node_i_][0])

        for i in range(0, 101):
            x = NODES_POSITIONS[node_i][0] + x_dist * i
            y = NODES_POSITIONS[node_i][1] + y_dist * i
            self.br.sendTransform((x, y, 0),
                                tf.transformations.quaternion_from_euler(0, 0, self.current_yaw - math.radians(90)),
                                rospy.Time.now(),
                                self.ns + "_kuka_mobile_base_link",
                                "world")
            time.sleep(0.01)

        yaw_dist_at_node_i_ = (NODES_RELATIONSHIPS[node_i][node_i_][1] - math.degrees(self.current_yaw)) / 100
        if yaw_dist_at_node_i_ != 0:
            x = NODES_POSITIONS[node_i_][0]
            y = NODES_POSITIONS[node_i_][1]
            for i in range(0, 101):
                yaw_i = self.current_yaw + math.radians(yaw_dist_at_node_i_) * i
                self.br.sendTransform((x, y, 0),
                                    tf.transformations.quaternion_from_euler(0, 0, yaw_i - math.radians(90)),
                                    rospy.Time.now(),
                                    self.ns + "_kuka_mobile_base_link",
                                    "world")
                time.sleep(0.01)
            self.current_yaw = math.radians(NODES_RELATIONSHIPS[node_i][node_i_][1])

        self.br.sendTransform((x, y, 0),
                            tf.transformations.quaternion_from_euler(0, 0, self.current_yaw - math.radians(90)),
                            rospy.Time.now(),
                            self.ns + "_kuka_mobile_base_link",
                            "world")

if __name__ == '__main__':
    navigator = RobotNavigator()
    navigator.move_base()
