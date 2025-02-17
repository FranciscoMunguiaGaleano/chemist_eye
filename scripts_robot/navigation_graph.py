#!/usr/bin/env python
import rospy
import random
import math
import heapq
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

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


def find_shortest_path(start, goal):
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


def draw_path(path, marker_array):
    path_marker = Marker()
    path_marker.header.frame_id = "world"
    path_marker.ns = "path"
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

def calculate_distances():
    distances = {}
    for node, neighbors in NODES_RELATIONSHIPS.items():
        for neighbor in neighbors:
            dist = math.sqrt((NODES_POSITIONS[node][0] - NODES_POSITIONS[neighbor][0]) ** 2 +
                             (NODES_POSITIONS[node][1] - NODES_POSITIONS[neighbor][1]) ** 2)
            distances[(node, neighbor)] = dist
    return distances

def create_markers(node_states):
    marker_array = MarkerArray()
    for node_id, position in NODES_POSITIONS.items():
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "nodes"
        marker.id = node_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.001
        
        color = COLOR_MAP[node_states[node_id]]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        marker_array.markers.append(marker)
    
    line_marker = Marker()
    line_marker.header.frame_id = "world"
    line_marker.header.stamp = rospy.Time.now()
    line_marker.ns = "lines"
    line_marker.id = 1000
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
            point1.z = 0.1
            
            point2 = Point()
            point2.x = NODES_POSITIONS[neighbor][0]
            point2.y = NODES_POSITIONS[neighbor][1]
            point2.z = 0.1
            
            line_marker.points.append(point1)
            line_marker.points.append(point2)
    
    marker_array.markers.append(line_marker)
    
    return marker_array

def marker_publisher(start_node, goal_node, velocity):
    rospy.init_node('node_marker_publisher', anonymous=True)
    pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  
    node_states = {node: 1 for node in NODES_POSITIONS.keys()}  # Assign random states
    path = find_shortest_path(24, 36)
    rospy.loginfo(path)
    
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        markers = create_markers(node_states)
        
        #if path and len(path) > 0:  # Ensure path has at least two points
        #    draw_path(path, marker_array)
        
        for marker in markers.markers:
            marker.pose.orientation.w = 1.0  # Ensure quaternion is initialized
            marker_array.markers.append(marker)
        
        pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Start and goal nodes, and velocity values can now be set via launch file
        start_node = rospy.get_param('~start_node', 1)
        goal_node = rospy.get_param('~goal_node', 36)
        velocity = rospy.get_param('~velocity', 0.2)
        
        marker_publisher(start_node, goal_node, velocity)
    except rospy.ROSInterruptException:
        pass

