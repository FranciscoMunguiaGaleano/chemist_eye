#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import pyautogui
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from chemist_eye.srv import RunBashScript  
import random
import ollama
import time
import base64
from io import BytesIO
import re
from typing import Tuple, Optional


CHEMIST_EYE_ONE_POSES = [[1, 1], [1, 2], [1, 3], [2, 1], [2, 2]]
CHEMIST_EYE_TWO_POSES = [[1, 1], [1, 2], [1, 3], [2, 1], [2, 2]]
CHEMIST_EYE_THREE_POSES = [[1, 1], [1, 2], [1, 3], [2, 1], [2, 2]]
VALID_NODES = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12, 13, 15, 16,17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38, 39]

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


def call_chemisteyeone_speech_service():
    # Wait for the service to be available
    rospy.wait_for_service('/run_speech_service_one')  # Make sure the service is available

    try:
        # Create a service proxy to call the service
        run_bash_script = rospy.ServiceProxy('/run_speech_service_one', RunBashScript)
        
        # Call the service and store the response
        response = run_bash_script()

        # Print the result
        if response.success:
            rospy.loginfo("Speech service one executed successfully.")
        else:
            rospy.logwarn(f"Failed to execute speech service one: {response.message}")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

class LlmDecisionMaker:
    def __init__(self):
        rospy.init_node('rviz_camera_publisher', anonymous=True)
        self.experiment = rospy.get_param('~exp', 'fire')
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/rviz_camera_view", Image, queue_size=10)
        self.warning_colour_one_pub = rospy.Publisher('cameraone/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_one_pub.publish("red")
        self.warning_colour_two_pub = rospy.Publisher('cameratwo/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_two_pub.publish("gray")
        self.warning_colour_three_pub = rospy.Publisher('camerathree/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_three_pub.publish("gray")
        self.control_robot_one_pub = rospy.Publisher('robot1/node', String, queue_size=10)  # Publish color data
        self.control_robot_two_pub = rospy.Publisher('robot2/node', String, queue_size=10)  # Publish color data
        self.control_robot_three_pub = rospy.Publisher('robot3/node', String, queue_size=10)  # Publish color data
        self.message_trigger_pub = rospy.Publisher("/slack_messages_trigger", String, queue_size = 10)
        self.message_trigger_pub.publish("False")
        self.event_description_pub = rospy.Publisher("/slack_event_description", String, queue_size = 10)
        self.event_description_pub.publish("No description available")
        self.rviz_view = None

        # Load cropping parameters from ROS parameters or default values
        self.x = rospy.get_param("~crop_x", 600)  # Default crop X position
        self.y = rospy.get_param("~crop_y", 120)  # Default crop Y position
        self.width = rospy.get_param("~crop_width", 1200)  # Default crop width
        self.height = rospy.get_param("~crop_height", 850)  # Default crop height

        #random.seed(22)
        
        time_stamp_limit_1_ = random.randint(100,160)
        time_stamp_limit_2_ = random.randint(200,300)
        time_stamp_limit_3_ = random.randint(150,160)
        time_stamp_limit_1 = 0
        time_stamp_limit_2 = 0
        time_stamp_limit_3 = 0
        self.camera_source = random.randint(1, 3)

        rate = rospy.Rate(10)  # Publish at 10Hz
        while not rospy.is_shutdown():
            self.publish_screenshot()
            if self.experiment == 'accident':
                if time_stamp_limit_1 <= time_stamp_limit_1_:
                    self.warning_colour_one_pub.publish("gray")
                else:
                    if self.camera_source == 1:
                        rospy.loginfo(f"🚨 Accident detected")
                        for i in range(0,10):
                            if self.camera_source == 1: 
                                self.warning_colour_one_pub.publish("red")
                            elif self.camera_source == 2:
                                self.warning_colour_two_pub.publish("red")
                            else:
                                self.warning_colour_three_pub.publish("red")
                                
                        time.sleep(1)
                        for i in range(0, 10):
                            self.publish_screenshot()
                        time.sleep(1)
                        while True:
                            #query = " The image is an RViz map view. In this map: Meeples represent people: Gray meeples indicate that everything is normal. Yellow meeples indicate that a person is not wearing PPE. Red meeples indicate that an accident has occurred. There are three KUKA robots on mobile bases (labeled as robot1, robot2 and robot3). Green dots represent locations where the robots can move. Numbers above the green dots correspond to specific movement nodes. If there is a red meeple, it means an accident has occurred at that location. Given this information, which destination nodes (numbers above the green dots possible nubers are: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12, 13, 15, 16,17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38 and 39) should be the best for robot1, robot2 and robot3 in order to keep the space clear from obstacles, ensuring help can reach the affected area efficiently?. Respond ONLY in the following format: ROBOT1: [X], ROBOT2: [Y]  ROBOT3: [Z] Replace X, Y, Z with the selected node numbers for robot1 robot2 and robot3, respectively. Do NOT include explanations or extra tex and avoid the robots to end too close from each other."
                            query = " The image is an RViz map view. In this map: Meeples represent people: Gray meeples indicate that everything is normal. Yellow meeples indicate that a person is not wearing PPE. Red meeples indicate that an accident has occurred. There are three KUKA robots on mobile bases (labeled as robot1, robot2 and robot3). Green dots represent locations where the robots can move. Numbers above the green dots correspond to specific movement nodes. If there is a red meeple, it means an accident has occurred at that location. Given this information, which destination nodes (numbers above the green dots possible nubers are: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12, 13, 15, 16,17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38 and 39) should be the best for robot1, robot2 and robot3 in order to keep the space clear from obstacles, ensuring help can reach the affected area efficiently?. Respond ONLY in the following format: ROBOT1: [X], ROBOT2: [Y]  ROBOT3: [Z] Replace X, Y, Z with the selected node numbers for robot1 robot2 and robot3, respectively. The lenght of [X] must be 1, the lenght of [Y] must be one and the lenght of [Z] must be one. Please avoid the robots to end close from each other or send them to the same nodes or to nodes that are close to each other."
                            answer = self.query_llm(self.rviz_view, query)
                            rospy.loginfo(f"LLM Response to serious accident: {answer}")
                            nodes = extract_robot_numbers(answer)
                            rospy.loginfo(f'The node numbers are: {nodes}')
                            try:
                                rospy.loginfo(f'Kuka 1 should go to: {nodes[0]}')
                                rospy.loginfo(f'kuka 2 should go to: {nodes[1]}')
                                rospy.loginfo(f'Kuka 3 should go to: {nodes[2]}')
                            except:
                                rospy.loginfo(f'Invalid locations')
                            if nodes[0] is not None and nodes[1] is not None and nodes[2] is not None and nodes[0] in VALID_NODES and nodes[1] in VALID_NODES and nodes[2] in VALID_NODES:
                                for i in range(0, 10):
                                    self.event_description_pub.publish(f"🚨 *A potential accident has been detected in the lab.* The nodes ChemistEye selected to move the robots based on the current incident are summarised as follows: {answer}")
                                for i in range(0, 10):
                                    self.message_trigger_pub.publish("True")
                                for i in range (0,10):
                                    self.control_robot_one_pub.publish(str(nodes[0]))
                                    self.control_robot_two_pub.publish(str(nodes[1]))
                                    self.control_robot_three_pub.publish(str(nodes[2]))
                                time.sleep(90)
                                for i in range(0, 10):
                                    self.publish_screenshot()
                                for i in range(0, 10):
                                    self.event_description_pub.publish(f"⚠️ *The following image displays the current state of the lab after the robots have been moved.*")
                                for i in range(0, 10):
                                    self.message_trigger_pub.publish("True")
                                break
                            else: 
                                rospy.loginfo("LLM failed to give a valid answer, querying again...")
                        while not rospy.is_shutdown():
                            pass
                    time_stamp_limit_1 = 0
                if time_stamp_limit_2 <= time_stamp_limit_2_:
                    self.warning_colour_two_pub.publish("gray")
                else:
                    if self.camera_source == 2:
                        rospy.loginfo(f"🚨 Accident detected")
                        for i in range(0,10):
                            if self.camera_source == 1: 
                                self.warning_colour_one_pub.publish("red")
                            elif self.camera_source == 2:
                                self.warning_colour_two_pub.publish("red")
                            else:
                                self.warning_colour_three_pub.publish("red")
                                
                        time.sleep(1)
                        for i in range(0, 10):
                            self.publish_screenshot()
                        time.sleep(1)
                        while True:
                            #query = " The image is an RViz map view. In this map: Meeples represent people: Gray meeples indicate that everything is normal. Yellow meeples indicate that a person is not wearing PPE. Red meeples indicate that an accident has occurred. There are three KUKA robots on mobile bases (labeled as robot1, robot2 and robot3). Green dots represent locations where the robots can move. Numbers above the green dots correspond to specific movement nodes. If there is a red meeple, it means an accident has occurred at that location. Given this information, which destination nodes (numbers above the green dots possible nubers are: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12, 13, 15, 16,17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38 and 39) should be the best for robot1, robot2 and robot3 in order to keep the space clear from obstacles, ensuring help can reach the affected area efficiently?. Respond ONLY in the following format: ROBOT1: [X], ROBOT2: [Y]  ROBOT3: [Z] Replace X, Y, Z with the selected node numbers for robot1 robot2 and robot3, respectively. Do NOT include explanations or extra tex and avoid the robots to end too close from each other."
                            query = " The image is an RViz map view. In this map: Meeples represent people: Gray meeples indicate that everything is normal. Yellow meeples indicate that a person is not wearing PPE. Red meeples indicate that an accident has occurred. There are three KUKA robots on mobile bases (labeled as robot1, robot2 and robot3). Green dots represent locations where the robots can move. Numbers above the green dots correspond to specific movement nodes. If there is a red meeple, it means an accident has occurred at that location. Given this information, which destination nodes (numbers above the green dots possible nubers are: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12, 13, 15, 16,17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38 and 39) should be the best for robot1, robot2 and robot3 in order to keep the space clear from obstacles, ensuring help can reach the affected area efficiently?. Respond ONLY in the following format: ROBOT1: [X], ROBOT2: [Y]  ROBOT3: [Z] Replace X, Y, Z with the selected node numbers for robot1 robot2 and robot3, respectively. The lenght of [X] must be 1, the lenght of [Y] must be one and the lenght of [Z] must be one. Please avoid the robots to end close from each other or send them to the same nodes or to nodes that are close to each other."
                            answer = self.query_llm(self.rviz_view, query)
                            rospy.loginfo(f"LLM Response to serious accident: {answer}")
                            nodes = extract_robot_numbers(answer)
                            rospy.loginfo(f'The node numbers are: {nodes}')
                            try:
                                rospy.loginfo(f'Kuka 1 should go to: {nodes[0]}')
                                rospy.loginfo(f'kuka 2 should go to: {nodes[1]}')
                                rospy.loginfo(f'Kuka 3 should go to: {nodes[2]}')
                            except:
                                rospy.loginfo(f'Invalid locations')
                            if nodes[0] is not None and nodes[1] is not None and nodes[2] is not None and nodes[0] in VALID_NODES and nodes[1] in VALID_NODES and nodes[2] in VALID_NODES:
                                for i in range(0, 10):
                                    self.event_description_pub.publish(f"🚨 *A potential accident has been detected in the lab.* The nodes ChemistEye selected to move the robots based on the current incident are summarised as follows: {answer}")
                                for i in range(0, 10):
                                    self.message_trigger_pub.publish("True")
                                for i in range (0,10):
                                    self.control_robot_one_pub.publish(str(nodes[0]))
                                    self.control_robot_two_pub.publish(str(nodes[1]))
                                    self.control_robot_three_pub.publish(str(nodes[2]))
                                time.sleep(90)
                                for i in range(0, 10):
                                    self.publish_screenshot()
                                for i in range(0, 10):
                                    self.event_description_pub.publish(f"⚠️ *The following image displays the current state of the lab after the robots have been moved.*")
                                for i in range(0, 10):
                                    self.message_trigger_pub.publish("True")
                                break
                            else: 
                                rospy.loginfo("LLM failed to give a valid answer, querying again...")
                        while not rospy.is_shutdown():
                            pass
                    time_stamp_limit_2 = 0
                if time_stamp_limit_3 <= time_stamp_limit_3_:
                    self.warning_colour_three_pub.publish("gray")
                else:
                    if self.camera_source == 3:
                        rospy.loginfo(f"🚨 Accident detected")
                        for i in range(0,10):
                            if self.camera_source == 1: 
                                self.warning_colour_one_pub.publish("red")
                            elif self.camera_source == 2:
                                self.warning_colour_two_pub.publish("red")
                            else:
                                self.warning_colour_three_pub.publish("red")
                                
                        time.sleep(1)
                        for i in range(0, 10):
                            self.publish_screenshot()
                        time.sleep(1)
                        while True:
                            #query = " The image is an RViz map view. In this map: Meeples represent people: Gray meeples indicate that everything is normal. Yellow meeples indicate that a person is not wearing PPE. Red meeples indicate that an accident has occurred. There are three KUKA robots on mobile bases (labeled as robot1, robot2 and robot3). Green dots represent locations where the robots can move. Numbers above the green dots correspond to specific movement nodes. If there is a red meeple, it means an accident has occurred at that location. Given this information, which destination nodes (numbers above the green dots possible nubers are: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12, 13, 15, 16,17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38 and 39) should be the best for robot1, robot2 and robot3 in order to keep the space clear from obstacles, ensuring help can reach the affected area efficiently?. Respond ONLY in the following format: ROBOT1: [X], ROBOT2: [Y]  ROBOT3: [Z] Replace X, Y, Z with the selected node numbers for robot1 robot2 and robot3, respectively. Do NOT include explanations or extra tex and avoid the robots to end too close from each other."
                            query = " The image is an RViz map view. In this map: Meeples represent people: Gray meeples indicate that everything is normal. Yellow meeples indicate that a person is not wearing PPE. Red meeples indicate that an accident has occurred. There are three KUKA robots on mobile bases (labeled as robot1, robot2 and robot3). Green dots represent locations where the robots can move. Numbers above the green dots correspond to specific movement nodes. If there is a red meeple, it means an accident has occurred at that location. Given this information, which destination nodes (numbers above the green dots possible nubers are: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ,12, 13, 15, 16,17, 18, 19, 20, 22, 24, 25, 26, 27, 28, 29, 30, 32, 33, 34, 35, 36, 37, 38 and 39) should be the best for robot1, robot2 and robot3 in order to keep the space clear from obstacles, ensuring help can reach the affected area efficiently?. Respond ONLY in the following format: ROBOT1: [X], ROBOT2: [Y]  ROBOT3: [Z] Replace X, Y, Z with the selected node numbers for robot1 robot2 and robot3, respectively. The lenght of [X] must be 1, the lenght of [Y] must be one and the lenght of [Z] must be one. Please avoid the robots to end close from each other or send them to the same nodes or to nodes that are close to each other."
                            answer = self.query_llm(self.rviz_view, query)
                            rospy.loginfo(f"LLM Response to serious accident: {answer}")
                            nodes = extract_robot_numbers(answer)
                            rospy.loginfo(f'The node numbers are: {nodes}')
                            try:
                                rospy.loginfo(f'Kuka 1 should go to: {nodes[0]}')
                                rospy.loginfo(f'kuka 2 should go to: {nodes[1]}')
                                rospy.loginfo(f'Kuka 3 should go to: {nodes[2]}')
                            except:
                                rospy.loginfo(f'Invalid locations')
                            if nodes[0] is not None and nodes[1] is not None and nodes[2] is not None and nodes[0] in VALID_NODES and nodes[1] in VALID_NODES and nodes[2] in VALID_NODES:
                                for i in range(0, 10):
                                    self.event_description_pub.publish(f"🚨 *A potential accident has been detected in the lab.* The nodes ChemistEye selected to move the robots based on the current incident are summarised as follows: {answer}")
                                for i in range(0, 10):
                                    self.message_trigger_pub.publish("True")
                                for i in range (0,10):
                                    self.control_robot_one_pub.publish(str(nodes[0]))
                                    self.control_robot_two_pub.publish(str(nodes[1]))
                                    self.control_robot_three_pub.publish(str(nodes[2]))
                                time.sleep(90)
                                for i in range(0, 10):
                                    self.publish_screenshot()
                                for i in range(0, 10):
                                    self.event_description_pub.publish(f"⚠️ *The following image displays the current state of the lab after the robots have been moved.*")
                                for i in range(0, 10):
                                    self.message_trigger_pub.publish("True")
                                break
                            else: 
                                rospy.loginfo("LLM failed to give a valid answer, querying again...")
                        while not rospy.is_shutdown():
                            pass
                    time_stamp_limit_3 = 0
                time_stamp_limit_1 += 1
                time_stamp_limit_2 += 1
                time_stamp_limit_3 += 1


            if self.experiment == 'ppe':
                if time_stamp_limit_1 <= time_stamp_limit_1_:
                    self.warning_colour_one_pub.publish("gray")
                else:
                    if self.camera_source == 1: 
                        rospy.loginfo(f"⚠️ A person has been detected not wearing PPE at ChemistEye 1 station for more than 10 minutes. ChemistEye is 🔊 issuing warnings and has frozen the robots to prevent any potential risks to the worker not complying with PPE requirements.")
                        for i in range(0,10):
                            if self.camera_source == 1: 
                                self.warning_colour_one_pub.publish("yellow")
                            elif self.camera_source == 2:
                                self.warning_colour_two_pub.publish("yellow")
                            else:
                                self.warning_colour_three_pub.publish("yellow")
                        time.sleep(10)
                        #Call warning
                        #Notify incident
                        for i in range(0, 10):
                                    self.publish_screenshot()
                        for i in range(0, 10):
                                    self.event_description_pub.publish(f"⚠️ A person has been detected not wearing PPE at ChemistEye 1 station for more than *10 minutes*. ChemistEye is 🔊 issuing warnings and has frozen the robots to prevent any potential risks to the worker not complying with PPE requirements.")
                        for i in range(0, 10):
                            self.message_trigger_pub.publish("True")
                    time_stamp_limit_1 = 0
                if time_stamp_limit_2 <= time_stamp_limit_2_:
                    self.warning_colour_two_pub.publish("gray")
                    #Call warning
                else:
                    if self.camera_source == 2: 
                        rospy.loginfo(f"⚠️ A person has been detected not wearing PPE at ChemistEye 2 station for more than 10 minutes. ChemistEye is 🔊 issuing warnings and has frozen the robots to prevent any potential risks to the worker not complying with PPE requirements.")
                        for i in range(0,10):
                            if self.camera_source == 1: 
                                self.warning_colour_one_pub.publish("yellow")
                            elif self.camera_source == 2:
                                self.warning_colour_two_pub.publish("yellow")
                            else:
                                self.warning_colour_three_pub.publish("yellow")
                        time.sleep(10)
                        time.sleep(20)
                        #Call warning
                        #Notify incident
                        for i in range(0, 10):
                                    self.publish_screenshot()
                        for i in range(0, 10):
                                    self.event_description_pub.publish(f"⚠️ A person has been detected not wearing PPE at ChemistEye 2 station for more than *10 minutes*. ChemistEye is 🔊 issuing warnings and has frozen the robots to prevent any potential risks to the worker not complying with PPE requirements.")
                        for i in range(0, 10):
                            self.message_trigger_pub.publish("True")
                        time_stamp_limit_2 = 0
                if time_stamp_limit_3 <= time_stamp_limit_3_:
                    self.warning_colour_three_pub.publish("gray")
                else:
                    if self.camera_source == 3:
                        rospy.loginfo(f"⚠️ A person has been detected not wearing PPE at ChemistEye 3 station for more than 10 minutes. ChemistEye is 🔊 issuing warnings and has frozen the robots to prevent any potential risks to the worker not complying with PPE requirements.")
                        for i in range(0,10):
                            if self.camera_source == 1: 
                                self.warning_colour_one_pub.publish("yellow")
                            elif self.camera_source == 2:
                                self.warning_colour_two_pub.publish("yellow")
                            else:
                                self.warning_colour_three_pub.publish("yellow")
                        time.sleep(10)
                        #Call warning
                        for i in range(0, 10):
                                    self.publish_screenshot()
                        for i in range(0, 10):
                                    self.event_description_pub.publish(f"⚠️ A person has been detected not wearing PPE at ChemistEye 3 station for more than *10 minutes*. ChemistEye is 🔊 issuing warnings and has frozen the robots to prevent any potential risks to the worker not complying with PPE requirements.")
                        for i in range(0, 10):
                            self.message_trigger_pub.publish("True")
                    time_stamp_limit_3 = 0

                time_stamp_limit_1 += 1
                time_stamp_limit_2 += 1
                time_stamp_limit_3 += 1
            rate.sleep()

    def crop_image(self, image):
        """Crops the image using defined x, y, width, and height."""
        h, w, _ = image.shape

        # Ensure crop dimensions are within bounds
        x1 = max(0, min(self.x, w))
        y1 = max(0, min(self.y, h))
        x2 = max(0, min(self.x + self.width, w))
        y2 = max(0, min(self.y + self.height, h))

        return image[y1:y2, x1:x2]

    def publish_screenshot(self):
        # Capture the full screen
        screenshot = pyautogui.screenshot()
        frame = np.array(screenshot)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Crop the image
        cropped_frame = self.crop_image(frame)
        self.rviz_view = cropped_frame.copy()

        # Publish as a ROS Image message
        msg = self.bridge.cv2_to_imgmsg(cropped_frame, encoding="bgr8")
        self.image_pub.publish(msg)

    def query_llm(self, image_data, query):
        try:
            if image_data is not None:
                rospy.loginfo(f"Querying LLM with image data of shape: {image_data.shape}")

                # Convert OpenCV image (NumPy array) to base64 string
                _, buffer = cv2.imencode('.jpg', image_data)
                image_base64 = base64.b64encode(buffer).decode('utf-8')

                response = ollama.chat(
                    model='llava:7b',
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

if __name__ == "__main__":
    LlmDecisionMaker()
