#!/usr/bin/env python

import rospy
import random
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime

class RandomImageSaver:
    def __init__(self):
        rospy.init_node('random_image_saver', anonymous=True)

        # Parameters
        self.save_path = rospy.get_param(
            '~save_path',
            '/media/francisco/New Volume/ROS_bags/raw_images'
        )

        self.image_prefix = rospy.get_param('~image_prefix', 'PPE_')
        self.camera = rospy.get_param('~camera', 1)

        # Topic selection
        if self.camera == 1:
            self.topic = '/camera/color/image_raw'
        else:
            self.topic = '/cameratwo/color/image_raw'

        rospy.loginfo(f"Subscribing to: {self.topic}")
        rospy.loginfo(f"Saving to: {self.save_path}")

        # Ensure directory exists
        os.makedirs(self.save_path, exist_ok=True)

        self.bridge = CvBridge()
        self.last_save_time = rospy.Time.now()

        # First random interval
        self.next_interval = self.get_random_interval()

        self.sub = rospy.Subscriber(self.topic, Image, self.callback)

    def get_random_interval(self):
        return random.uniform(0.5, 2.0)

    def callback(self, msg):
        now = rospy.Time.now()
        elapsed = (now - self.last_save_time).to_sec()

        if elapsed >= self.next_interval:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"{self.image_prefix}{timestamp}.jpg"
                filepath = os.path.join(self.save_path, filename)

                cv2.imwrite(filepath, cv_image)

                rospy.loginfo(f"Saved: {filepath}")

                # Reset timer
                self.last_save_time = now
                self.next_interval = self.get_random_interval()

            except Exception as e:
                rospy.logerr(f"Error saving image: {e}")

if __name__ == '__main__':
    try:
        RandomImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass