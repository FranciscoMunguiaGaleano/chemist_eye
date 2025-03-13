#!/usr/bin/env python

import rospy
import requests
import configparser
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from slack_sdk import WebClient
from slack_sdk.errors import SlackApiError
from slack_bolt import App
from PIL import Image as pil

class EventNotifier:
    def __init__(self):
        rospy.init_node("event_notifier")

        # Load Slack tokens from bot.ini
        config = configparser.ConfigParser()
        config.read("/home/francisco/catkin_ws/src/chemist_eye/config/bot.ini")
        #print(config.keys())

        self.app_token = config["KEYS"]["AppToken"]
        self.bot_token = config["KEYS"]["BotToken"]
        self.signing_secret = config["KEYS"]["SlackSignature"]
        self.slack_channel = config["INFO"]["BroadcastChannel"]

        # Initialize Slack client
        self.app = App(token=self.bot_token, signing_secret=self.signing_secret)
        self.client = WebClient(token=self.bot_token)

        self.bridge = CvBridge()
        self.latest_image = None

        # Subscribers
        self.image_sub = rospy.Subscriber("/rviz_camera_view", Image, self.image_callback)
        self.message_trigger_sub = rospy.Subscriber("/slack_messages_trigger", String, self.trigger_callback)
        self.event_description_sub = rospy.Subscriber("/slack_event_description", String, self.event_description_callback)
        
        self.message_trigger = False
        self.event_description = ""
        rospy.loginfo("Event Notifier Node Ready!")

    def trigger_callback(self, msg):
        if msg.data == "True":
            self.message_trigger = True

    def event_description_callback(self, msg):
        self.event_description = msg.data

    def image_callback(self, msg):
        """ Store the latest image from ROS topic """
        self.latest_image = msg

    def send_to_slack(self):
        """ Send event description and latest image to Slack """
        if self.latest_image is None:
            rospy.logwarn("No image received yet!")
            return

        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")

            # Save image to a temporary file
            temp_image_path = "/home/francisco/catkin_ws/src/chemist_eye/temp/event_image.jpg"
            cv2.imwrite(temp_image_path, cv_image)
            
            # Send message 
            self.client.chat_postMessage(
                channel=self.slack_channel,
                text=f"{self.event_description}"
            )

            # Upload image to Slack
            self.client.files_upload_v2(
                channel=self.slack_channel,
                initial_comment="",
                file=temp_image_path,
            )
            self.message_trigger = False
            rospy.loginfo("Event sent to Slack successfully!")

            # Remove the temporary image file
            #os.remove(temp_image_path)

        except SlackApiError as e:
            rospy.logerr(f"Slack API Error: {e.response['error']}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        """ Run the node and accept event descriptions from command line """
        while not rospy.is_shutdown():
            #event_description = input("Enter event description: ")
            if self.message_trigger:
                self.send_to_slack()


#@notifier.app.event("app_mention")
#def mention_handler(body, say):
#    channel = body["event"]["channel"]  # Extract the channel ID
#    say(channel)  # Send a message to the same channel

if __name__ == "__main__":
    notifier = EventNotifier()
    notifier.run()
