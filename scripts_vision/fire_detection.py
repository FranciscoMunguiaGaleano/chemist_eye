#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
import random
import ollama
import time
import base64
from io import BytesIO
import cv2
import re
from typing import Tuple, Optional
from cv_bridge import CvBridge
import random

MAX_TEMPERATURE = 200  # Maximum threshold for fire detection
VALID_NODES = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38, 39]  # Valid node numbers


def extract_robot_numbers(text: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
    # Regex to match numbers inside brackets or standalone numbers after "ROBOTx:"
    patterns = [r"ROBOT1: \[?(\d+(?:,\s*\d+)*)\]?", 
                r"ROBOT2: \[?(\d+(?:,\s*\d+)*)\]?", 
                r"ROBOT3: \[?(\d+(?:,\s*\d+)*)\]?"]
    
    matches = [re.search(pattern, text, re.DOTALL) for pattern in patterns]
    
    def get_number(match: Optional[re.Match]) -> Optional[int]:
        if not match:
            return None
        numbers = list(map(int, re.findall(r'\d+', match.group(1))))  # Extract all numbers
        return numbers[0] if numbers else None  # Return first number if exists, else None
    
    return tuple(get_number(match) for match in matches)

class FireDetectionNode:
    def __init__(self):
        rospy.init_node('fire_detection_node', anonymous=True)
        self.max_temp = rospy.get_param('~MAX_TEMP', 500)
        self.llm_model = rospy.get_param('~llm_model', 'llava:7b')
        self.map_mode = rospy.get_param('~map_mode', '3D')  # default to 3D
        self.c_mode = rospy.get_param('~c_mode', 'C1') # Condition mode C1, C2, C3
        self.exp_number = rospy.get_param('~exp_number', '1') 
        self.rand_mode = rospy.get_param('~rand_mode', False)
        self.random_sample = random.sample([1, 2], 1)

        # Decide which image topic to subscribe to
        if self.map_mode == '2D':
            rospy.Subscriber("/maptwo2", Image, self.rviz_view_callback)
        else:
            rospy.Subscriber("/rviz_camera_view", Image, self.rviz_view_callback)

        # Subscribe to available nodes
        self.available_nodes = []
        rospy.Subscriber("/available_nodes", String, self.available_nodes_callback)

        # Publishers for RViz markers
        self.marker_pub1 = rospy.Publisher('fire_marker_1', Marker, queue_size=10)
        self.text_pub1 = rospy.Publisher('fire_marker_text_1', Marker, queue_size=10)

        self.marker_pub2 = rospy.Publisher('fire_marker_2', Marker, queue_size=10)
        self.text_pub2 = rospy.Publisher('fire_marker_text_2', Marker, queue_size=10)

        self.fire_call_pub = rospy.Publisher('fire_emergency', String, queue_size=10) 
        self.fire_call_pub.publish("False")
        self.warning_colour_one_pub = rospy.Publisher('cameraone/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_two_pub = rospy.Publisher('cameratwo/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_three_pub = rospy.Publisher('camerathree/warning_color_topic', String, queue_size=10)  # Publish color data

        # Subscribers to temperature topics
        rospy.Subscriber('camerair/max_temperature', Float32, self.temperature_callback_1, ('IR_camera_link', 1, self.marker_pub1, self.text_pub1))
        rospy.Subscriber('camerair_two/max_temperature', Float32, self.temperature_callback_2, ('IR_cameratwo_link', 2, self.marker_pub2, self.text_pub2))
        # rospy.Subscriber("/rviz_camera_view", Image, self.rviz_view_callback)

        # Publishers for robot control
        self.control_robot_one_pub = rospy.Publisher('robot1/node', String, queue_size=10)
        self.control_robot_two_pub = rospy.Publisher('robot2/node', String, queue_size=10)
        self.control_robot_three_pub = rospy.Publisher('robot3/node', String, queue_size=10)

        self.message_trigger_pub = rospy.Publisher("/slack_messages_trigger", String, queue_size=10)
        self.event_description_pub = rospy.Publisher("/slack_event_description", String, queue_size=10)

        self.message_trigger_pub.publish("False")
        self.event_description_pub.publish("No description available")

        self.latest_image = None
        self.bridge = CvBridge()
        self.temp1 = 0
        self.temp2 = 0
        self.freeze_temp = False
        rospy.loginfo("🔥 Fire detection node started. Monitoring temperature with RViz markers...")
        while not rospy.is_shutdown():
            if not self.rand_mode:
                if self.temp1*1 > self.max_temp or self.temp2*1>self.max_temp:
                    self.freeze_temp = True
                    for i in range(0, 10):
                        self.warning_colour_one_pub.publish("yellow")
                        self.warning_colour_two_pub.publish("yellow")
                        self.warning_colour_three_pub.publish("yellow")
                        time.sleep(0.1)
                    rospy.loginfo(f"🔥 Potential fire detected! Temperature exceeded {self.max_temp}°C!")
                    self.handle_fire_detection()
            else:
                if self.random_sample == 1:
                    if self.temp1*1 > self.max_temp:
                        self.freeze_temp = True
                        for i in range(0, 10):
                            self.warning_colour_one_pub.publish("yellow")
                            self.warning_colour_two_pub.publish("yellow")
                            self.warning_colour_three_pub.publish("yellow")
                            time.sleep(0.1)
                        rospy.loginfo(f"🔥 Potential fire detected! Temperature exceeded {self.max_temp}°C!")
                        self.handle_fire_detection()
                else:
                    if self.temp2*1>self.max_temp:
                        self.freeze_temp = True
                        for i in range(0, 10):
                            self.warning_colour_one_pub.publish("yellow")
                            self.warning_colour_two_pub.publish("yellow")
                            self.warning_colour_three_pub.publish("yellow")
                            time.sleep(0.1)
                        rospy.loginfo(f"🔥 Potential fire detected! Temperature exceeded {self.max_temp}°C!")
                        self.handle_fire_detection()
    
    def available_nodes_callback(self, msg):
        try:
            self.available_nodes = msg.data
            #rospy.loginfo(f"Updated available nodes: {self.available_nodes}")
        except Exception as e:
            rospy.logwarn(f"Failed to parse available nodes: {e}")

        
    def get_color_from_temperature(self, temperature):
        """ Returns an RGB color transitioning from blue (cold) to red (hot). """
        normalized_temp = min(1.0, max(0.0, temperature / self.max_temp))  # Clamp between 0 and 1
        r = normalized_temp                                                # More red as it gets hotter
        g = 0.0                                                            # Green removed for a cleaner transition
        b = 1.0 - normalized_temp                                          # Blue fades out
        return r, g, b

    def create_sphere_marker(self, temp, frame_id, marker_id):
        """ Creates an RViz sphere marker based on temperature. """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "fire_detection"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.scale.x = 0.9  # Sphere size
        marker.scale.y = 0.9
        marker.scale.z = 0.9

        r, g, b = self.get_color_from_temperature(temp)  # Get color from temperature
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.6  # Transparency for better visibility

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.2  # Sphere's height

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker

    def create_text_marker(self, temp, frame_id, marker_id):
        """ Creates a text marker displaying the temperature above the sphere. """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "fire_detection_text"
        marker.id = marker_id + 100  # Unique ID offset for text
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.scale.z = 0.4  # Text size

        marker.color.r = 1.0  # White text
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Fully visible

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.8  # Slightly above the sphere

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.text = f"{temp:.1f}°C"  # Display temperature

        return marker

    def temperature_callback_1(self, msg, args):
        """ Callback for temperature sensors, publishing markers in RViz. """
        frame_id, marker_id, marker_pub, text_pub = args
        temp = msg.data
        self.temp1 = temp
        if self.freeze_temp:
            temp = self.max_temp

        # Publish sphere marker
        sphere_marker = self.create_sphere_marker(temp, frame_id, marker_id)
        marker_pub.publish(sphere_marker)

        # Publish temperature text marker
        text_marker = self.create_text_marker(temp, frame_id, marker_id)
        text_pub.publish(text_marker)
    def temperature_callback_2(self, msg, args):
        """ Callback for temperature sensors, publishing markers in RViz. """
        frame_id, marker_id, marker_pub, text_pub = args
        temp = msg.data
        self.temp2 = temp
        if self.freeze_temp:
            temp = self.max_temp

        # Publish sphere marker
        sphere_marker = self.create_sphere_marker(temp, frame_id, marker_id)
        marker_pub.publish(sphere_marker)

        # Publish temperature text marker
        text_marker = self.create_text_marker(temp, frame_id, marker_id)
        text_pub.publish(text_marker)

    def rviz_view_callback(self, msg):
        """ Converts ROS Image message to OpenCV format and stores it. """
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # Convert to OpenCV image
            #rospy.loginfo("Received an image and converted it successfully.")
        except Exception as e:
            rospy.logerr(f"Error converting ROS Image to OpenCV: {e}")

    def query_llm(self, image_data, query):
        try:
            if image_data is not None:
                rospy.loginfo(f"Querying LLM with image data of shape: {image_data.shape}")

                # Convert OpenCV image (NumPy array) to base64 string
                _, buffer = cv2.imencode('.jpg', image_data)
                image_base64 = base64.b64encode(buffer).decode('utf-8')

                response = ollama.chat(
                    model = self.llm_model,
                    messages=[
                        {
                            'role': 'user',
                            'content': query,
                            'images': [image_base64],  # ✅ Now passing base64 string instead of NumPy array
                        }
                    ],
                )

                return response.get('message', {}).get('content', '').strip().upper()
        except Exception as e:
            rospy.logerr(f"Error querying LLM: {e}")
            return None

    def handle_fire_detection(self):
        """ Handles actions when a fire is detected. """
        while True:
            if self.c_mode == 'C1':
                if self.map_mode == '2D':
                    query = (
                    f"The image is an RViz map view showing people as triangles: gray means normal, yellow means missing PPE, and red indicates an accident. "
                    f"There are three mobile KUKA robots (R1, R2, R3) on the map (orange squares). "
                    f"Green dots represent possible robot navigation nodes. "
                    f"Blue and red circles mark areas of detected temperature; a potential fire has been detected. "
                    f"Based on the image, suggest safe and distant navigation nodes for each robot to avoid the fire. "
                    f"The nodes must be selected from the list provided above, and each robot should move to a unique node. "
                    f"Respond in this exact format: ROBOT1: [X], ROBOT2: [Y], ROBOT3: [Z], where X, Y, and Z are node numbers. "
                    f"Avoid assigning the same or nearby nodes to different robots."
                    f"The node number should be 0 when there is not necessity to move a robot.")
                else:
                    query = (
                    f"The image is an RViz map view showing people as meeples: gray means normal, yellow means missing PPE, and red indicates an accident. "
                    f"There are three mobile KUKA robots (Robot1, Robot2, Robot3) on the map."
                    f"Green dots represent possible robot navigation nodes. "
                    f"Blue and red spheres mark areas of detected temperature; a potential fire has been detected. "
                    f"Based on the image, suggest safe and distant navigation nodes for each robot to avoid the fire. "
                    f"The nodes must be selected from the list provided above, and each robot should move to a unique node. "
                    f"Respond in this exact format: ROBOT1: [X], ROBOT2: [Y], ROBOT3: [Z], where X, Y, and Z are node numbers. "
                    f"Avoid assigning the same or nearby nodes to different robots."
                    f"The node number should be 0 when there is not necessity to move a robot.")
            if self.c_mode == 'C2':
                if self.map_mode == '2D':
                    query = (
                    f"The image is an RViz map view showing people as triangles: gray means normal, yellow means missing PPE, and red indicates an accident. "
                    f"There are three mobile KUKA robots (R1, R2, R3) on the map (orange squares). "
                    f"Green dots represent possible robot navigation nodes, which only available numbers to pick are: {VALID_NODES}. "
                    f"Blue and red circles mark areas of detected temperature; a potential fire has been detected. "
                    f"Based on the image, suggest safe and distant navigation nodes for each robot to avoid the fire. "
                    f"The nodes must be selected from the list provided above, and each robot should move to a unique node. "
                    f"Respond in this exact format: ROBOT1: [X], ROBOT2: [Y], ROBOT3: [Z], where X, Y, and Z are node numbers. "
                    f"Avoid assigning the same or nearby nodes to different robots."
                    f"The node number should be 0 when there is not necessity to move a robot.")
                else:
                    query = (
                    f"The image is an RViz map view showing people as meeples: gray means normal, yellow means missing PPE, and red indicates an accident. "
                    f"There are three mobile KUKA robots (Robot1, Robot2, Robot3) on the map."
                    f"Green dots represent possible robot navigation nodes, which only available numbers to pick are: {VALID_NODES}. "
                    f"Blue and red spheres mark areas of detected temperature; a potential fire has been detected. "
                    f"Based on the image, suggest safe and distant navigation nodes for each robot to avoid the fire. "
                    f"The nodes must be selected from the list provided above, and each robot should move to a unique node. "
                    f"Respond in this exact format: ROBOT1: [X], ROBOT2: [Y], ROBOT3: [Z], where X, Y, and Z are node numbers. "
                    f"Avoid assigning the same or nearby nodes to different robots."
                    f"The node number should be 0 when there is not necessity to move a robot.")
            if self.c_mode == 'C3':
                time.sleep(10)
                if self.map_mode == '2D':
                    query = (
                    f"The image is an RViz map view showing people as triangles: gray means normal, yellow means missing PPE, and red indicates an accident. "
                    f"There are three mobile KUKA robots (R1, R2, R3) on the map (orange squares). "
                    f"Green dots represent possible robot navigation nodes. "
                    f"Red circles mark areas of detected temperature; at the moment of this query, a potential fire has been detected. "
                    f"Based on the image, suggest safe and distant navigation nodes for each robot to avoid the fire (RED CIRCLES). "
                    f"The nodes must be selected from the list provided above, and each robot should move to a unique node. "
                    f"Respond in this exact format: ROBOT1: [X], ROBOT2: [Y], ROBOT3: [Z], where X, Y, and Z are node numbers, the only available node numbers to pick are:: [{self.available_nodes}]."
                    f"Avoid assigning the same nodes to different robots."
                    f"The node number should be 0 when there is not necessity to move a robot.")
                else:
                    query = (
                    f"The image is an RViz map view showing people as meeples: gray means normal, yellow means missing PPE, and red indicates an accident. "
                    f"There are three mobile KUKA robots (Robot1, Robot2, Robot3) on the map."
                    f"Green dots represent possible robot navigation nodes."
                    f"Red spheres mark areas of detected temperature; at the moment of this query, a potential fire has been detected. "
                    f"Based on the image, suggest safe and distant navigation nodes for each robot to avoid the fire (RED SPHERES). "
                    f"The nodes must be selected from the list provided above, and each robot should move to a unique node. "
                    f"Respond in this exact format: ROBOT1: [X], ROBOT2: [Y], ROBOT3: [Z], where X, Y, and Z are node numbers, the only available node numbers to pick are: [{self.available_nodes}]. "
                    f"Avoid assigning the same nodes to different robots."
                    f"The node number should be 0 when there is not necessity to move a robot.")

            rospy.loginfo(f"LLM Query: {query}")
            answer = self.query_llm(cv2.resize(self.latest_image, (0, 0), fx=0.9, fy=0.9), query)
            rospy.loginfo(f"LLM Response to serious accident: {answer}")
            nodes = extract_robot_numbers(answer)
            rospy.loginfo(f'The node numbers are: {nodes} from {self.available_nodes}')
            try:
                rospy.loginfo(f'Kuka 1 should go to: {nodes[0]}')
                rospy.loginfo(f'kuka 2 should go to: {nodes[1]}')
                rospy.loginfo(f'Kuka 3 should go to: {nodes[2]}')
                self.move_robots(nodes, answer)
                break
            except:
                rospy.loginfo(f'Invalid locations')

    def move_robots(self, nodes, answer):
        """ Move robots to selected nodes. """
        for i in range(0, 10):
            self.event_description_pub.publish(f"🔥 *A potential fire has been detected* [{self.llm_model}] {answer}.  Experiment: {self.exp_number}")
        for i in range(0, 10):
            self.message_trigger_pub.publish("True")
        for i in range (0,10):
            if nodes[0] in VALID_NODES:
                self.control_robot_one_pub.publish(str(nodes[0]))
            if nodes[1] in VALID_NODES:
                self.control_robot_two_pub.publish(str(nodes[1]))
            if nodes[2] in VALID_NODES:
                self.control_robot_three_pub.publish(str(nodes[2]))
        time.sleep(90)
        for i in range(0, 10):
            self.event_description_pub.publish(f"⚠️ *The following image displays the current state of the lab after the robots have been moved.* Experiment[{self.llm_model}]: {self.exp_number}")
        for i in range(0, 10):
            self.message_trigger_pub.publish("True")

    def fire_detection_node(self):
        """ Main node for fire detection with RViz visualization. """
        rospy.spin()


if __name__ == '__main__':
    try:
        fire_detection = FireDetectionNode()
        fire_detection.fire_detection_node()
    except rospy.ROSInterruptException:
        pass
