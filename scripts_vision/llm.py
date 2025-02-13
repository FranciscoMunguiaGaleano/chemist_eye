#!/usr/bin/env python

import ollama
import rospy
import os
import cv2
from cv_bridge import CvBridge
from chemist_eye.srv import RunBashScript 
from sensor_msgs.msg import Image
from std_msgs.msg import String
import time
from ultralytics import YOLO
import numpy as np
from PIL import Image as PilImage

IMGS_TEMP_1_PATH = os.path.join(os.path.dirname(__file__), '..', 'temp', 'imgone.jpg')
IMGS_TEMP_2_PATH = os.path.join(os.path.dirname(__file__), '..', 'temp', 'imgtwo.jpg')
IMGS_TEMP_3_PATH = os.path.join(os.path.dirname(__file__), '..', 'temp', 'imgthree.jpg')
DETECTION_CONFIDENCE = 0.8

class VLLMClassification():
    def __init__(self):
        rospy.init_node('llm_classification_node', anonymous=True)
        self.model = YOLO("yolov8n.pt", verbose=False)  # YOLOv8 nano model
        self.target_classes = [0]
        self.bridge = CvBridge()
        self.image_one_latest = None
        self.image_two_latest = None
        self.image_three_latest = None
        self.image_sub_one = rospy.Subscriber('camera/color/image_raw', Image, self.imageone_callback)
        self.image_sub_two = rospy.Subscriber('cameratwo/color/image_raw', Image, self.imagetwo_callback)
        self.image_sub_three = rospy.Subscriber('camerathree/color/image_raw', Image, self.imagethree_callback)
        self.warning_colour_one_pub = rospy.Publisher('cameraone/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_one_pub.publish("gray")
        self.warning_colour_two_pub = rospy.Publisher('cameratwo/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_two_pub.publish("gray")
        self.warning_colour_three_pub = rospy.Publisher('camerathree/warning_color_topic', String, queue_size=10)  # Publish color data
        self.warning_colour_three_pub.publish("gray")
    def imageone_callback(self, msg):
        try:
            self.image_one_latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Error processing image one: {e}")

    def imagetwo_callback(self, msg):
        try:
            self.image_two_latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Error processing image two: {e}")

    def imagethree_callback(self, msg):
        try:
            self.image_three_latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Error processing image three: {e}")

    def count_persons(self, img_path):
        num_persons = 0
        try:
            if not os.path.exists(img_path):
                rospy.logerr(f"Image file not found: {img_path}")
                return num_persons

            # Load image using OpenCV
            frame = cv2.imread(img_path)
            if frame is None:
                rospy.logerr(f"Failed to load image: {img_path}")
                return num_persons

            # Convert BGR (OpenCV default) to RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Convert to PIL Image (if required by YOLO model)
            frame_pil = PilImage.fromarray(frame)

            # Run YOLO prediction
            results = self.model.predict(source=frame_pil, show=False, task='pose', verbose=False)

            detections = results[0].boxes  # Access bounding boxes

            if detections is not None:
                for detection in detections:
                    xyxy = detection.xyxy[0].tolist()  # Bounding box [x1, y1, x2, y2]
                    conf = detection.conf[0]  # Confidence
                    cls = int(detection.cls[0])  # Class ID
                    label = self.model.names[cls]  # Class name

                    # Filter detections for target classes and confidence >= 80%
                    if cls in self.target_classes and conf >= DETECTION_CONFIDENCE:
                        num_persons += 1
            return num_persons

        except Exception as e:
            rospy.logerr(f"Error querying YOLO: {e}")
            return num_persons
    
    def process_image(self, img_path, speech_service, camera_name):
        rospy.loginfo(f"Processing image from {camera_name}")
        num_persons = self.count_persons(img_path)
        rospy.loginfo(f"Number of persons detected from {camera_name}: {num_persons}")
        if num_persons == 1:
            # Check if the person is lying on the floor
            query = 'Is there a person lying, kneeling, prone or on all fours on the ground? Only reply with yes or no.'
            answer = query_llm(img_path, query)
            rospy.loginfo(f"LLM Response ({camera_name} - Serious Accident): {answer}")

            if not answer or "YES" in answer:
                self.publish_marker_colours("red", camera_name)
                rospy.loginfo(f"A person potentially accidented has been detected in the image from {camera_name}. Triggering emergency notification service.")
                self.publish_marker_colours("red", camera_name)
                # set marker as red
                time.sleep(100)
                
                #set markers as 1,1,1 (RGB)
            # Check if the person is wearing lab coat
            query = 'Is the person wearing lab coat? Only reply with yes or no.'
            answer = query_llm(img_path, query)
            rospy.loginfo(f"LLM Response ({camera_name} - Wearing Lab coat): {answer}")

            if not answer or "NO" in answer:
                rospy.loginfo(f"Lab coat not detected for the person in the image from {camera_name}. Triggering speech service.")
                self.publish_marker_colours("yellow", camera_name)
                call_speech_service(speech_service)
                #set markers as yellow
                
            else:
                self.publish_marker_colours("gray", camera_name)
                #set markers as 1,1,1 (RGB)
        elif num_persons > 1:
            query = 'Are all the persons in the image wearing lab coat? Only reply with yes or no.'
            answer = query_llm(img_path, query)
            rospy.loginfo(f"LLM Response ({camera_name} - Wearing Lab coat): {answer}")

            if not answer or "NO" in answer:
                rospy.loginfo(f"Lab coats not detected for the persons in the image from {camera_name}. Triggering speech service.")
                call_speech_service(speech_service)
        else:
            self.publish_marker_colours("gray", camera_name)
            
    def publish_marker_colours(self,colour, camera_name):
        if camera_name == 'Camera One':
            self.warning_colour_one_pub.publish(colour)
        elif camera_name == 'Camera Two':
            self.warning_colour_two_pub.publish(colour)
        elif camera_name == 'Camera Three':
            self.warning_colour_three_pub.publish(colour)
        else:
            rospy.logerr("Not a valid camera.")
        return
def call_speech_service(service_name):
    rospy.wait_for_service(service_name)
    try:
        run_bash_script = rospy.ServiceProxy(service_name, RunBashScript)
        response = run_bash_script()
        if response.success:
            rospy.loginfo(f"Speech service {service_name} executed successfully.")
        else:
            rospy.logwarn(f"Failed to execute {service_name}: {response.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

    
def query_llm(img_path, query):
    try:
        if not os.path.exists(img_path):
            rospy.logerr(f"Image file not found: {img_path}")
            return None

        with open(img_path, 'rb') as img_file:
            image_data = img_file.read()
            rospy.loginfo(f"Querying LLM with image data of size: {len(image_data)} bytes")

            response = ollama.chat(
                model='llava:7b',
                messages=[
                    {
                        'role': 'user',
                        'content': query,
                        'images': [image_data],
                    }
                ],
            )

        return response.get('message', {}).get('content', '').strip().upper()
    except Exception as e:
        rospy.logerr(f"Error querying LLM: {e}")
        return None


if __name__ == '__main__':
    try:
        node = VLLMClassification()
        rate = rospy.Rate(0.1)  # 0.1 Hz (10 seconds interval)
        while not rospy.is_shutdown():
            try:

                if node.image_one_latest is not None:
                    cv2.imwrite(IMGS_TEMP_1_PATH, node.image_one_latest)
                    node.process_image(IMGS_TEMP_1_PATH, '/run_speech_service_one', 'Camera One')

                if node.image_two_latest is not None:
                    cv2.imwrite(IMGS_TEMP_2_PATH, node.image_two_latest)
                    node.process_image(IMGS_TEMP_2_PATH, '/run_speech_service_two', 'Camera Two')

                if node.image_three_latest is not None:
                    cv2.imwrite(IMGS_TEMP_3_PATH, node.image_three_latest)
                    node.process_image(IMGS_TEMP_3_PATH, '/run_speech_service_three', 'Camera Three')
            except:
                rospy.logwarn("Chemist eye not streaming...")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
