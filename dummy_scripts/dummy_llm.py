#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import pyautogui
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import random
import ollama
import time
import base64
from io import BytesIO


CHEMIST_EYE_ONE_POSES = [[1, 1], [1, 2], [1, 3], [2, 1], [2, 2]]
CHEMIST_EYE_TWO_POSES = [[1, 1], [1, 2], [1, 3], [2, 1], [2, 2]]
CHEMIST_EYE_THREE_POSES = [[1, 1], [1, 2], [1, 3], [2, 1], [2, 2]]

class LlmDecisionMaker:
    def __init__(self):
        rospy.init_node('rviz_camera_publisher', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/rviz_camera_view", Image, queue_size=10)
        self.warning_colour_one_pub = rospy.Publisher('cameraone/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_one_pub.publish("red")
        self.warning_colour_two_pub = rospy.Publisher('cameratwo/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_two_pub.publish("gray")
        self.warning_colour_three_pub = rospy.Publisher('camerathree/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_three_pub.publish("gray")

        self.rviz_view = None

        # Load cropping parameters from ROS parameters or default values
        self.x = rospy.get_param("~crop_x", 600)  # Default crop X position
        self.y = rospy.get_param("~crop_y", 120)  # Default crop Y position
        self.width = rospy.get_param("~crop_width", 1200)  # Default crop width
        self.height = rospy.get_param("~crop_height", 850)  # Default crop height

        random.seed(22)
        
        time_stamp_limit_1_ = random.randint(50,160)
        time_stamp_limit_2_ = random.randint(100,999)
        time_stamp_limit_3_ = random.randint(100,999)
        time_stamp_limit_1 = 0
        time_stamp_limit_2 = 0
        time_stamp_limit_3 = 0

        rate = rospy.Rate(10)  # Publish at 10Hz
        while not rospy.is_shutdown():
            self.publish_screenshot()
            if time_stamp_limit_1 < time_stamp_limit_1_:
                self.warning_colour_one_pub.publish("gray")
            else:
                for i in range(0,10):
                    self.warning_colour_one_pub.publish("red")
                time.sleep(1)
                self.publish_screenshot()
                query = "The image is an RViz map view. In this map: Meeples represent people: Gray meeples indicate that everything is normal. Yellow meeples indicate that a person is not wearing PPE. Red meeples indicate that an accident has occurred. There are three KUKA robots on mobile bases. Green dots represent locations where the robots can move. Numbers above the green dots correspond to specific movement nodes. If there is a red meeple, it means an accident has occurred at that location. Given this information, which destination nodes (numbers above the green dots) should be the best for robots in order to keep the space clear from obstacles, ensuring help can reach the affected area efficiently?"
                query = " The image is an RViz map view. In this map: Meeples represent people: Gray meeples indicate that everything is normal. Yellow meeples indicate that a person is not wearing PPE. Red meeples indicate that an accident has occurred. There are three KUKA robots on mobile bases. Green dots represent locations where the robots can move. Numbers above the green dots correspond to specific movement nodes. If there is a red meeple, it means an accident has occurred at that location. Given this information, which destination nodes (numbers above the green dots) should be the best for robots in order to keep the space clear from obstacles, ensuring help can reach the affected area efficiently?. Respond ONLY in the following format: NODE LIST: [X, Y, Z]  Replace X, Y, Z with the selected node numbers. Do NOT include explanations or extra text."
                answer = self.query_llm(self.rviz_view, query)
                rospy.loginfo(f"LLM Response to serious accident: {answer}")
                #matches = re.findall(r' (\d+)', answer)
                # Convert matches to integers
                #node_numbers = list(map(int, matches))

                #print("Extracted movement nodes:", node_numbers)
                time_stamp_limit_1 = 0
                #Call llm and query about best robot positions for that given accident, then call service to change node numbers
                #Notify the accident throgh slack
                #Once the nodes have been reached stop all robots
            self.warning_colour_two_pub.publish("gray")
            self.warning_colour_three_pub.publish("gray")
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
