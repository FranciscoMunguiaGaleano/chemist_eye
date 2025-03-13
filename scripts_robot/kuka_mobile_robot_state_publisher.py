#!/usr/bin/env python
import rospy
import tf
import math
import heapq
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import time
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

class RobotNavigator:
    def __init__(self):
        rospy.init_node('mobile_base_tf_broadcaster')
        self.ns = rospy.get_param('~ns_name', "robot1")
        self.warning_colour_one_sub = rospy.Subscriber('/cameraone/warning_color_topic', String, self.colourone_callback)
        self.warning_colour_two_sub = rospy.Subscriber('/cameratwo/warning_color_topic', String, self.colourtwo_callback)
        self.warning_colour_three_sub = rospy.Subscriber('/camerathree/warning_color_topic', String, self.colourthree_callback)
        self.control_robot_one_sub = rospy.Subscriber('node', String, self.set_nav_goal)  # Publish color data
        self.fire_call_sub = rospy.Subscriber('fire_emergency', String, self.fire_callback)  # Publish color data
        self.text_marker_pub = rospy.Publisher('text_marker_kuka', Marker, queue_size=10)
        self.path_pub = rospy.Publisher(self.ns + '_visualization_path_marker_array', MarkerArray, queue_size=10)
        
        rospy.loginfo(F"Node inisialied")
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)
        self.current_yaw = math.radians(90)
        self.markers_colours_one = 'gray'
        self.markers_colours_two = 'gray'
        self.markers_colours_three = 'gray'
        self.emergency = False
        self.navigation_goal = None

    def fire_callback(self, msg):
        if msg.data == "True":
            self.emergency = True
    
    def set_nav_goal(self, msg):
        self.navigation_goal = int(msg.data)
        #rospy.loginfo(F"Navigating {self.ns} to node: {msg}")
    def colourone_callback(self, msg):
        self.markers_colours_one = msg.data
        if self.markers_colours_one != 'gray':
            self.emergency = True
        if self.markers_colours_three == 'gray' and self.markers_colours_two == 'gray' and self.markers_colours_one == 'gray':
            self.emergency = False
    def colourtwo_callback(self, msg):
        self.markers_colours_two = msg.data
        #rospy.loginfo(F"Colour changed to {msg}")
        if self.markers_colours_two != 'gray':
            self.emergency = True
        if self.markers_colours_three == 'gray' and self.markers_colours_two == 'gray' and self.markers_colours_one == 'gray':
            self.emergency = False
    def colourthree_callback(self, msg):
        self.markers_colours_three = msg.data
        #rospy.loginfo(F"Colour changed to {msg}")
        if self.markers_colours_three != 'gray':
            self.emergency = True
        if self.markers_colours_three == 'gray' and self.markers_colours_two == 'gray' and self.markers_colours_one == 'gray':
            self.emergency = False
        

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

    def follow_path(self, path, v_max):
        try:
            for i in range(0, len(path) - 1):
                self.move_to_node(path[i], path[i + 1], v_max)
                while self.emergency:
                    #rospy.loginfo('Emergency detecte idleing robots')
                    self.idle(path[i + 1])
                    if self.navigation_goal is not None:
                        path_ = self.find_shortest_path(path[i + 1], self.navigation_goal)
                        # navigate path
                        if len(path_) > 0 and automatic_routine:
                            rospy.loginfo(f'New Path found for {self.ns}: {path_}')
                            self.draw_path(path_, marker_array)
                            self.path_pub.publish(marker_array)
                            for j in range(0, len(path_) - 1):
                                self.move_to_node(path_[j], path_[j + 1], v_max)
                            while self.emergency:
                                self.idle(self.navigation_goal)
                                # Wait until slack gives a continue ...
                        else:
                            while self.emergency:
                                self.idle(path[i + 1])

                        
                        
                    
            for i in range(len(path) - 1, 0, -1):
                self.move_to_node(path[i], path[i - 1], v_max)
                while self.emergency:
                    #rospy.loginfo('Emergency detecte idleing robots')
                    self.idle(path[i - 1])
                    if self.navigation_goal is not None:
                        path_ = self.find_shortest_path(path[i - 1], self.navigation_goal)
                        # navigate path
                        if len(path_) > 0 and automatic_routine:
                            rospy.loginfo(f'New Path found for {self.ns}: {path_}')
                            self.draw_path(path_, marker_array)
                            self.path_pub.publish(marker_array)
                            for j in range(0, len(path_) - 1):
                                self.move_to_node(path_[j], path_[j + 1], v_max)
                            while self.emergency:
                                self.idle(self.navigation_goal)
                                # Wait until slack gives a continue ...
                        else:
                            while self.emergency:
                                self.idle(path[i - 1])

        except Exception as e:
            rospy.loginfo(f'[Error] {e}')
    
    def calculate_total_time(self, v_max, x1, x2, y1, y2):
        return (math.sqrt((x2 - x1)**2+(y2 - y1)**2)/v_max)
    
    def idle(self, node_i):
        #self.current_yaw 
        x = NODES_POSITIONS[node_i][0]
        y = NODES_POSITIONS[node_i][1]
        self.br.sendTransform((x, y, 0),
                            tf.transformations.quaternion_from_euler(0, 0, self.current_yaw - math.radians(90)),
                            rospy.Time.now(),
                            self.ns + "_kuka_mobile_base_link",
                            "world")
        self.add_text_marker(x,y)


    def move_to_node(self, node_i, node_i_, v_max):
        x_dist = (NODES_POSITIONS[node_i_][0] - NODES_POSITIONS[node_i][0]) / 100
        y_dist = (NODES_POSITIONS[node_i_][1] - NODES_POSITIONS[node_i][1]) / 100
        yaw_dist_at_node_i = (NODES_RELATIONSHIPS[node_i][node_i_][0] - math.degrees(self.current_yaw)) / 100
        
        if yaw_dist_at_node_i != 0:
            x = NODES_POSITIONS[node_i][0]
            y = NODES_POSITIONS[node_i][1]
            self.add_text_marker(x,y)
            for i in range(0, 101):
                yaw_i = self.current_yaw + math.radians(yaw_dist_at_node_i) * i
                self.br.sendTransform((x, y, 0),
                                    tf.transformations.quaternion_from_euler(0, 0, yaw_i - math.radians(90)),
                                    rospy.Time.now(),
                                    self.ns + "_kuka_mobile_base_link",
                                    "world")
                
                time.sleep(0.1)
                
            self.current_yaw = math.radians(NODES_RELATIONSHIPS[node_i][node_i_][0])
        t_i = rospy.Time.now().to_sec()
        t_i_ = t_i + self.calculate_total_time(v_max, NODES_POSITIONS[node_i][0], NODES_POSITIONS[node_i_][0], NODES_POSITIONS[node_i][1], NODES_POSITIONS[node_i_][1])
        d = math.sqrt(NODES_POSITIONS[node_i][0]**2 + NODES_POSITIONS[node_i][1]) 
        for i in range(0, 101):
            x = NODES_POSITIONS[node_i][0] + x_dist * i
            y = NODES_POSITIONS[node_i][1] + y_dist * i
            self.add_text_marker(x,y)
            dt = t_i - t_i_
            d = d + v_max*dt
            t_i_ = t_i
            #rospy.loginfo(f'Distance: {d}')
            #rospy.loginfo(f'{t_i}:{t_i_}: Delta t: {dt}')
            self.br.sendTransform((x, y, 0),
                                tf.transformations.quaternion_from_euler(0, 0, self.current_yaw - math.radians(90)),
                                rospy.Time.now(),
                                self.ns + "_kuka_mobile_base_link",
                                "world")
            
            
            time.sleep(0.01)
            t_i = rospy.Time.now().to_sec()

        yaw_dist_at_node_i_ = (NODES_RELATIONSHIPS[node_i][node_i_][1] - math.degrees(self.current_yaw)) / 100
        if yaw_dist_at_node_i_ != 0:
            x = NODES_POSITIONS[node_i_][0]
            y = NODES_POSITIONS[node_i_][1]
            self.add_text_marker(x,y)
            for i in range(0, 101):
                yaw_i = self.current_yaw + math.radians(yaw_dist_at_node_i_) * i
                self.br.sendTransform((x, y, 0),
                                    tf.transformations.quaternion_from_euler(0, 0, yaw_i - math.radians(90)),
                                    rospy.Time.now(),
                                    self.ns + "_kuka_mobile_base_link",
                                    "world")
                #self.add_text_marker(x,y)
                time.sleep(0.01)
            self.current_yaw = math.radians(NODES_RELATIONSHIPS[node_i][node_i_][1])

        self.br.sendTransform((x, y, 0),
                            tf.transformations.quaternion_from_euler(0, 0, self.current_yaw - math.radians(90)),
                            rospy.Time.now(),
                            self.ns + "_kuka_mobile_base_link",
                            "world")
        self.add_text_marker(x,y)

    def add_text_marker(self, x, y):
        # Create a Marker for the text
        text_marker = Marker()
        text_marker.header.frame_id = "world"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "text_labels"  # Namespace for the text markers
        text_marker.id = 0  # You can use a counter or any unique ID for each marker
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # Set the position of the text marker above the robot
        text_marker.pose.position = Point(x, y, 3.0)  # Slightly above the robot (adjust the z-value)
        
        # Set the text you want to display
        text_marker.text = self.ns  # This will show the name of the robot (self.ns)
        
        # Set the size and color of the text
        text_marker.scale.z = 0.5  # Text size (adjust as necessary)
        text_marker.color.r = 1.0  # Red
        text_marker.color.g = 1.0  # Green
        text_marker.color.b = 1.0  # Blue
        text_marker.color.a = 1.0  # Full opacity
        
        # Publish the text marker
        self.text_marker_pub.publish(text_marker)

if __name__ == '__main__':
    navigator = RobotNavigator()
    ns = rospy.get_param('~ns_name', "robot1")
    start_node = rospy.get_param('~start_node', 24)
    goal_node = rospy.get_param('~goal_node', 36)
    automatic_routine = rospy.get_param('~automatic', False)
    v_max = rospy.get_param('~max_vel', 0.1)
    
        
    path = navigator.find_shortest_path(start_node, goal_node)
    marker_array = MarkerArray()
    navigator.draw_path(path, marker_array)
    #for i in range(0,10):
    navigator.path_pub.publish(marker_array)
    x = NODES_POSITIONS[start_node][0]
    y = NODES_POSITIONS[start_node][1]
    navigator.add_text_marker(x,y)

    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        #rospy.loginfo(F"here {ns} {path}, {automatic_routine}")
        #if ns == 'robot1':
        #    automatic_routine = True
        if len(path) > 0 and automatic_routine:
            navigator.draw_path(path, marker_array)
            navigator.path_pub.publish(marker_array)
            navigator.follow_path(path, v_max)
            #pass
        else:
            x = NODES_POSITIONS[start_node][0]
            y = NODES_POSITIONS[start_node][1]
            navigator.add_text_marker(x,y)
            yaw = math.radians(90)
            navigator.br.sendTransform((x, y, 0),
                                    tf.transformations.quaternion_from_euler(0, 0, yaw),
                                    rospy.Time.now(),
                                    navigator.ns + "_kuka_mobile_base_link",
                                    "world")
            navigator.add_text_marker(x,y)
        navigator.rate.sleep()
