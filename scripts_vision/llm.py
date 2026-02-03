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
import random
import base64

IMGS_TEMP_1_PATH = os.path.join(os.path.dirname(__file__), '..', 'temp', 'imgone.jpg')
IMGS_TEMP_2_PATH = os.path.join(os.path.dirname(__file__), '..', 'temp', 'imgtwo.jpg')
IMGS_TEMP_3_PATH = os.path.join(os.path.dirname(__file__), '..', 'temp', 'imgthree.jpg')
DATASET_WHITE_PPE = os.path.join(os.path.dirname(__file__), '..', 'temp/WHITE/PPE')
DATASET_WHITE_NONPPE = os.path.join(os.path.dirname(__file__), '..', 'temp/WHITE/NONPPE')
DATASET_BLUE_PPE = os.path.join(os.path.dirname(__file__), '..', 'temp/BLUE/PPE')
DATASET_BLUE_NONPPE = os.path.join(os.path.dirname(__file__), '..', 'temp/BLUE/NONPPE')
DATASET_GRAY_PPE = os.path.join(os.path.dirname(__file__), '..', 'temp/GRAY/PPE')
DATASET_GRAY_NONPPE = os.path.join(os.path.dirname(__file__), '..', 'temp/GRAY/NONPPE')
DATASET_GREEN_PPE = os.path.join(os.path.dirname(__file__), '..', 'temp/GREEN/PPE')
DATASET_GREEN_NONPPE = os.path.join(os.path.dirname(__file__), '..', 'temp/GREEN/NONPPE')
DATASET_PRONE = os.path.join(os.path.dirname(__file__), '..', 'temp/PRONE')
DATASET_NONPRONE = os.path.join(os.path.dirname(__file__), '..', 'temp/NONPRONE')
DATASET_STANDING = os.path.join(os.path.dirname(__file__), '..', 'temp/STANDING')
DATASET_NOTSTANDING = os.path.join(os.path.dirname(__file__), '..', 'temp/NOTSTANDING')
DATASET_PPE_NOT_CLASSIFIED = os.path.join(os.path.dirname(__file__), '..', 'temp/PPE_NOT_CLASSIFIED')
DATASET_STANDING_NOT_CLASSIFIED = os.path.join(os.path.dirname(__file__), '..', 'temp/STANDING_NOT_CLASSIFIED')
DETECTION_CONFIDENCE = 0.8

responses_to_invalid = []


class VLLMClassification():
    def __init__(self):
        rospy.init_node('llm_classification_node', anonymous=True)
        self.chemist_eye = rospy.get_param('~chemist_eye', 'all')
        self.prone = rospy.get_param('~prone', True)
        self.ppe = rospy.get_param('~ppe', True)
        self.save_dataset = rospy.get_param('~datasetppe', "WHITE") # WHITE, BLUE, GREEN, GRAY
        self.produce_dataset = rospy.get_param('~producedataset', False) 
        self.only_dataset = rospy.get_param('~onlydataset', False) 
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

    def get_strict_yes_no_response(self, img_path, query, llm, max_retries=3):
        """
        Ask the LLM a question and retry if the answer is not strictly 'YES' or 'NO'.
        """
        #rospy.loginfo(f"Attempt {max_retries+1} response: {answer}")
        for attempt in range(max_retries):
            answer = query_llm(img_path, query, llm)
            rospy.loginfo(f"Attempt {attempt+1} response: {answer}")
            if "YES" in answer or "NO" in answer:
                return answer
        rospy.loginfo("Warning: LLM gave ambiguous answer after retries. Defaulting to NO.")
        return "NO" 
    
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
        #return 1
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
    def process_image_ds(self, img_path, speech_service, camera_name):
        rospy.loginfo(f"Processing image from {camera_name}")

        num_persons = self.count_persons(img_path)
        rospy.loginfo(f"Number of persons detected from {camera_name}: {num_persons}")

        # Generate a unique filename with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")  # Format: YYYYMMDD_HHMMSS
        if self.only_dataset and num_persons == 1:
            if self.ppe:
                if self.produce_dataset:
                        ppe_img_path = os.path.join(DATASET_PPE_NOT_CLASSIFIED, f"ppe_{timestamp}.jpg")
                        try:
                            cv2.imwrite(ppe_img_path, cv2.imread(img_path))
                        except Exception as e:
                            rospy.loginfo(f"ppe_{timestamp}.jpg")
                            rospy.loginfo(f"{prone_img_path}")
                            rospy.loginfo(f"error here{e}")
            if self.prone:
                if self.produce_dataset:
                        prone_img_path = os.path.join(DATASET_STANDING_NOT_CLASSIFIED, f"prone_{timestamp}.jpg")
                        try:
                            cv2.imwrite(prone_img_path, cv2.imread(img_path))
                        except Exception as e:
                            rospy.loginfo(f"prone_{timestamp}.jpg")
                            rospy.loginfo(f"{prone_img_path}")
                            rospy.loginfo(f"error here{e}")
            return


        if num_persons == 1:
            # Check if the person is lying on the floor
            if self.prone:
                llm = 'llava-phi3:latest'
                q1 = 'What is the person doing?'
                answer = query_llm(img_path, q1, llm)
                rospy.loginfo(f"LLM Response ({camera_name} - Serious Accident): {answer}")
                if 'KNEELING' in answer or 'SITTING' in answer or 'CROUCHING' in answer or 'BENDING OVER' in answer or 'SQUATTING DOWN' in answer or 'LYING' in answer:
                    for i in range(0,10):
                        if camera_name == "Camera One": 
                            self.warning_colour_one_pub.publish("red")
                        elif camera_name == "Camera Two":
                            self.warning_colour_two_pub.publish("red")
                        else:
                            self.warning_colour_three_pub.publish("red")
                    rospy.loginfo(f"A person potentially accidented has been detected in the image from {camera_name}. Triggering emergency notification service.")
                elif 'WALKING' in answer or 'WALKS' in answer or 'STANDING' in answer or 'CHECKING' in answer or 'EXAMINING' in answer or 'LOOKING' in answer or 'WORKING' in answer:
                    print('The person is standing or walking')
                else:
                    print(f"Ambiguous answer: {answer}")
                    print("Warning: LLM gave ambiguous answer after retries. Defaulting to NO to avoid false positives.")
                if self.produce_dataset:
                    nonprone_img_path = os.path.join(DATASET_NOTSTANDING, f"prone_{timestamp}.jpg")
                    try:
                        cv2.imwrite(nonprone_img_path, cv2.imread(img_path))
                    except Exception as e:
                        rospy.loginfo(f"prone_{timestamp}.jpg")
                        rospy.loginfo(f"{prone_img_path}")
                        rospy.loginfo(f"error here{e}")

            # Check if the person is wearing a lab coat
            if self.ppe:
                llm = "llava-phi3:latest"
                q1 = "What is the person wearing?"
                answer = query_llm(img_path, q1, llm)
                
                rospy.loginfo(f"LLM Response ({camera_name} - Wearing Lab coat): {answer}")

                if "WHITE" in answer or "LAB COAT" in answer or "COAT" in answer:
                    self.publish_marker_colours("gray", camera_name)
                    if self.produce_dataset:
                        if self.save_dataset == "WHITE":
                            save_path = os.path.join(DATASET_WHITE_PPE, f"ppe_{timestamp}.jpg")
                        elif self.save_dataset == "BLUE":
                            save_path = os.path.join(DATASET_BLUE_PPE, f"ppe_{timestamp}.jpg")
                        elif self.save_dataset == "GREEN":
                            save_path = os.path.join(DATASET_GREEN_PPE, f"ppe_{timestamp}.jpg")
                        else:
                            save_path = os.path.join(DATASET_GRAY_PPE, f"ppe_{timestamp}.jpg")
                        cv2.imwrite(save_path, cv2.imread(img_path))
                else:
                    rospy.loginfo(f"Lab coat not detected for the person in the image from {camera_name}. Triggering speech service.")
                    self.publish_marker_colours("yellow", camera_name)
                    call_speech_service(speech_service)
                    if self.produce_dataset:
                        if self.save_dataset == "WHITE":
                            save_path = os.path.join(DATASET_WHITE_NONPPE, f"nonppe_{timestamp}.jpg")
                        elif self.save_dataset == "BLUE":
                            save_path = os.path.join(DATASET_BLUE_NONPPE, f"nonppe_{timestamp}.jpg")
                        elif self.save_dataset == "GREEN":
                            save_path = os.path.join(DATASET_GREEN_NONPPE, f"nonppe_{timestamp}.jpg")
                        else:
                            save_path = os.path.join(DATASET_GRAY_NONPPE, f"nonppe_{timestamp}.jpg")

                        cv2.imwrite(save_path, cv2.imread(img_path))
                
                    
    
    def process_image(self, img_path, speech_service, camera_name):
        rospy.loginfo(f"Processing image from {camera_name}")
        num_persons = self.count_persons(img_path)
        rospy.loginfo(f"Number of persons detected from {camera_name}: {num_persons}")
        if num_persons == 1:
            # Check if the person is lying on the floor
            if self.prone:
                llm = 'llava-phi3:latest'
                q1 = 'What is the person doing? ONLY reply with YES or NO.'
                answer = query_llm(img_path, query)
                rospy.loginfo(f"LLM Response ({camera_name} - Serious Accident): {answer}")

                answer = self.get_strict_yes_no_response(img_path, q1, llm)
                if 'KNEELING' in answer or 'SITTING' in answer or 'CROUCHING' in answer or 'BENDING OVER' in answer or 'SQUATTING DOWN' in answer or 'LYING' in answer:
                    self.publish_marker_colours("red", camera_name)
                    rospy.loginfo(f"A person potentially accidented has been detected in the image from {camera_name}. Triggering emergency notification service.")
                    rospy.loginfo(f"🚨 Accident detected")
                   
                elif 'WALKING' in answer or 'WALKS' in answer or 'STANDING' in answer or 'CHECKING' in answer or 'EXAMINING' in answer or 'LOOKING' in answer or 'WORKING' in answer:
                    print('The person is standing or walking')
                else:
                    print(f"Ambiguous answer: {answer}")
                    print("Warning: LLM gave ambiguous answer after retries. Defaulting to NO to avoid false positives.")
                
                #set markers as 1,1,1 (RGB)
            # Check if the person is wearing lab coat
            if self.ppe:
                query = 'Is the person wearing lab coat? ONLY reply with YES or NO'
                answer = query_llm(img_path, query)
                rospy.loginfo(f"LLM Response ({camera_name} - Wearing Lab coat): {answer}")

                if not answer or "NO" in answer:
                    rospy.loginfo(f"Lab coat not detected for the person in the image from {camera_name}. Triggering speech service.")
                    self.publish_marker_colours("yellow", camera_name)
                    call_speech_service(speech_service)
                    #set markers as yellow
                    if self.produce_dataset:
                        if self.save_dataset == "WHITE":
                            # save img_path in DATASET_WHITE_NONPPE
                            pass
                        elif self.save_dataset == "BLUE":
                            # save img_path in DATASET_BLUE_NONPPE
                            pass
                        elif self.save_dataset == "GREEN":
                            # save img_path in DATASET_GREEN_NONPPE
                            pass
                        else:
                            # save img_path in DATASET_GRAY_NONPPE
                            pass
                    
                else:
                    self.publish_marker_colours("gray", camera_name)
                    if self.produce_dataset:
                        if self.save_dataset == "WHITE":
                            # save img_path in DATASET_WHITE_PPE
                            pass
                        elif self.save_dataset == "BLUE":
                            # save img_path in DATASET_BLUE_PPE
                            pass
                        elif self.save_dataset == "GREEN":
                            # save img_path in DATASET_GREEN_PPE
                            pass
                        else:
                            # save img_path in DATASET_GRAY_PPE
                            pass
                #set markers as 1,1,1 (RGB)
        elif num_persons > 1:
            query = 'Are all the persons in the image wearing lab coat? Only reply with yes or no. DO NOT give  any further explanaitons'
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

    
def query_llm(img_path, query, llm):
    try:
        if not os.path.exists(img_path):
            print(f"Image file not found: {img_path}")
            return None

        with open(img_path, 'rb') as img_file:
            image_data = img_file.read()
            print(f"Querying LLM with image data of size: {len(image_data)} bytes")
            #'llava-phi3:latest'
            #'llama:7b'
            # deepseek-r1:1.5b
            response = ollama.chat(
                model= llm,
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
        print(f"Error querying LLM: {e}")
        return None
if __name__ == '__main__':
    try:
        node = VLLMClassification()
        try:
            rospy.loginfo(F"📸: {node.chemist_eye}")
        except Exception as e:
            rospy.loginfo(F"📸 Chemist eye not streaming error at the beginning: {e}")
        rate = rospy.Rate(0.1)  # 0.1 Hz (10 seconds interval)
        # 2, 3, 5, 7, 11, 13, 17, 19, 23, 29
        #time.sleep(1)
        cameras = ["camera1", "camera2", "camera3"]
        delays = [0.2, 0.3, 0.5, 0.7, 1.1, 1.9, 1.7, 2.3, 2.9]
        while not rospy.is_shutdown():
            try:
                time.sleep(random.choice(delays))
                #node.chemist_eye = random.choice(cameras)
                rospy.loginfo("Entering the deep seek here")
                if node.image_one_latest is not None and (node.chemist_eye == 'camera1' or node.chemist_eye == 'all'):
                    cv2.imwrite(IMGS_TEMP_1_PATH, node.image_one_latest)
                    node.process_image_ds(IMGS_TEMP_1_PATH, '/run_speech_service_one', 'Camera One')

                elif node.image_two_latest is not None and (node.chemist_eye == 'camera2' or node.chemist_eye == 'all'):
                    rospy.loginfo("Entering the deep seek")
                    cv2.imwrite(IMGS_TEMP_2_PATH, node.image_two_latest)
                    node.process_image_ds(IMGS_TEMP_2_PATH, '/run_speech_service_two', 'Camera Two')

                elif node.image_three_latest is not None and (node.chemist_eye == 'camera3' or node.chemist_eye == 'all'):
                    cv2.imwrite(IMGS_TEMP_3_PATH, node.image_three_latest)
                    node.process_image_ds(IMGS_TEMP_3_PATH, '/run_speech_service_three', 'Camera Three')
            except Exception as e:
                rospy.loginfo(F"📸 Chemist eye not streaming error: {e}")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
