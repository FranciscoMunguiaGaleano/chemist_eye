#!/usr/bin/env python
import rospy
import random
import math
import heapq
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, String

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

EXIT_LOCATIONS = [[-10.5, 1], [-8.8, 17]]


class MarkerManager:
    def __init__(self):
        rospy.init_node('node_marker_publisher', anonymous=True)
        self.pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        
        self.rate = rospy.Rate(1)
        self.node_states = {node: 1 for node in NODES_POSITIONS.keys()}  # Default state

    @staticmethod
    def calculate_distances():
        distances = {}
        for node, neighbors in NODES_RELATIONSHIPS.items():
            for neighbor in neighbors:
                dist = math.sqrt((NODES_POSITIONS[node][0] - NODES_POSITIONS[neighbor][0]) ** 2 +
                                 (NODES_POSITIONS[node][1] - NODES_POSITIONS[neighbor][1]) ** 2)
                distances[(node, neighbor)] = dist
        return distances

    def create_markers(self):
        marker_array = MarkerArray()

        # Create node markers
        for node_id, position in NODES_POSITIONS.items():
            # Cylinder marker for nodes
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "nodes"
            marker.id = node_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.1  # Slightly above ground
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.001

            color = COLOR_MAP[self.node_states[node_id]]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]

            marker_array.markers.append(marker)

            # Text marker to display node number
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "node_labels"
            text_marker.id = node_id + 1000  # Unique ID for text markers
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = position[0]
            text_marker.pose.position.y = position[1]
            text_marker.pose.position.z = 0.5  # Slightly above the node
            text_marker.scale.z = 0.4  # Size of the text
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = str(node_id)  # Assign node ID as text

            marker_array.markers.append(text_marker)

        for i, exit_pos in enumerate(EXIT_LOCATIONS):
            exit_marker = Marker()
            exit_marker.header.frame_id = "world"
            exit_marker.header.stamp = rospy.Time.now()
            exit_marker.ns = "exit_labels"
            exit_marker.id = 3000 + i  # Unique ID for exits
            exit_marker.type = Marker.TEXT_VIEW_FACING
            exit_marker.action = Marker.ADD
            exit_marker.pose.position.x = exit_pos[0]
            exit_marker.pose.position.y = exit_pos[1]
            exit_marker.pose.position.z = 0.5  # Above ground level
            exit_marker.scale.z = 0.5  # Adjust text size
            exit_marker.color.r = 1.0  # Red text for visibility
            exit_marker.color.g = 0.0
            exit_marker.color.b = 0.0
            exit_marker.color.a = 1.0
            exit_marker.text = "EXIT"  # Label as EXIT

            marker_array.markers.append(exit_marker)

        # Line markers for connections
        line_marker = Marker()
        line_marker.header.frame_id = "world"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = "lines"
        line_marker.id = 2000
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.01
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8

        for node, neighbors in NODES_RELATIONSHIPS.items():
            for neighbor in neighbors:
                point1 = Point()
                point1.x = NODES_POSITIONS[node][0]
                point1.y = NODES_POSITIONS[node][1]
                point1.z = 0.1  # Slightly above ground

                point2 = Point()
                point2.x = NODES_POSITIONS[neighbor][0]
                point2.y = NODES_POSITIONS[neighbor][1]
                point2.z = 0.1  # Slightly above ground

                line_marker.points.append(point1)
                line_marker.points.append(point2)

        marker_array.markers.append(line_marker)

        return marker_array


    def publish_markers(self):
        while not rospy.is_shutdown():
            marker_array = self.create_markers()
            for marker in marker_array.markers:
                marker.pose.orientation.w = 1.0 
            self.pub.publish(marker_array)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        manager = MarkerManager()
        manager.publish_markers()
    except rospy.ROSInterruptException:
        pass

