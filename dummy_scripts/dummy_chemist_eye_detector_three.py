#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
import tf
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Float32MultiArray, String
import time
from chemist_eye.srv import RunBashScript  
import tf_conversions
import os
import yaml
from scipy.stats import norm
import random

BOUNDED_DISTANCE = 8.0
CONFIG_FILE = '../config/cameraone_tf_conf.yaml'
DETECTION_CONFIDENCE = 0.7
WORKERS = [[-2, 2.5]]

class ExperimentsNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_detection_node', anonymous=True)
        self.camera_height = 1.5
        self.load_config()

        # Subscriptions
        self.warning_colour_one_sub = rospy.Subscriber('camerathree/warning_color_topic', String, self.colour_callback)

        # Publisher for TF transforms
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Publisher for RViz markers
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        
        # Distance tensor
        self.distance_tensor = None
        self.markers_colours = "gray"
        self.rate = rospy.Rate(10)
        self.pos_x, self.pos_z = random.choice(WORKERS)
        self.direction = True
        self.change = 0

    def colour_callback(self, msg):
        self.markers_colours = msg.data
        #if msg.data == "red":
        #    rospy.loginfo(F"🚨 Meeple colour changed to {msg}")

    def load_config(self):
        config_path = os.path.join(os.path.dirname(__file__), CONFIG_FILE)
        if os.path.exists(config_path):
            with open(config_path, 'r') as file:
                config_data = yaml.load(file, Loader=yaml.FullLoader)
            # Apply the configuration if it's valid
            if 'translation' in config_data and 'rotation' in config_data:
                translation = config_data['translation']
                rotation = config_data['rotation']
                self.camera_height=float(translation['z'])
                rospy.loginfo(f"Loaded configuration from {config_path}")
        else:
            rospy.logerr(f"No configuration file found at {config_path}")

    def broadcast_dummy(self):
        x_real, y_real, z_real = 2.5, 1, 3
        if self.markers_colours != "red":
            if self.direction:
                self.pos_x = self.pos_x + x_real*0.005 + random.uniform(-0.02, 0.02)
                self.pos_z = self.pos_z + z_real*0.005 + random.uniform(-0.02, 0.02)
            else:
                self.pos_x = self.pos_x - x_real*0.005 + random.uniform(-0.02, 0.02)
                self.pos_z = self.pos_z - z_real*0.005 + random.uniform(-0.02, 0.02) 
            if self.change == 10:
                self.direction = not self.direction
                self.change = 0
            self.change += 1
        self.broadcast_tf(self.pos_x, y_real, self.pos_z, 3)


    def broadcast_tf(self, x, y, z, human_id):
        """
        Broadcast a TF for the human position, with the camera frame rotated by 90 degrees around the X-axis.
        Each person gets a unique frame based on the human_id.
        """
        tf_msg = TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "camerathree_link"  # Parent frame
        tf_msg.child_frame_id = f"human_{human_id}_frame"  # Unique frame for each person

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = -(1.3) / 2 + self.camera_height
        tf_msg.transform.translation.z = z
        
        quaternion = tf_conversions.transformations.quaternion_from_euler(1.57079, 0.0, 0.0)
        # Rotation quaternion for 90 degrees around the X-axis
        tf_msg.transform.rotation.x = quaternion[0]
        tf_msg.transform.rotation.y = quaternion[1]
        tf_msg.transform.rotation.z = quaternion[2]
        tf_msg.transform.rotation.w = quaternion[3]

        # Send the transform
        self.tf_broadcaster.sendTransform(
            (tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z),
            (tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w),
            rospy.Time.now(),
            tf_msg.child_frame_id,
            tf_msg.header.frame_id
        )
        self.update_markers(0.0, 0.0, 0.0, human_id)

    def update_markers(self, x, y, z, human_id):
        """
        Create/update the markers for each human's body (cylinder for body, sphere for head and shoulders).
        """
        # Y is fixed for the markers to prevent displacement
        y = -1.5  # Keep Y fixed

        if self.markers_colours == "gray":
            R = 1.0
            G = 1.0
            B = 1.0
        elif self.markers_colours == "yellow":         
            R = 1.0
            G = 1.0
            B = 0.0
        elif self.markers_colours == "red":
            R = 1.0
            G = 0.0
            B = 0.0
        else:
            R = 1.0
            G = 1.0
            B = 1.0
        # Create the cylinder (body)
        cylinder_marker = Marker()
        cylinder_marker.header.frame_id = f"human_{human_id}_frame"
        cylinder_marker.header.stamp = rospy.Time.now()
        cylinder_marker.ns = f"human_markers_{human_id}"
        cylinder_marker.id = human_id * 3 +1  # Unique ID for each human (body)
        cylinder_marker.type = Marker.CYLINDER
        cylinder_marker.action = Marker.ADD
        cylinder_marker.pose.position.x = 0
        cylinder_marker.pose.position.y = 0
        cylinder_marker.pose.position.z = 0
        cylinder_marker.scale.x = 0.5  # Cylinder width
        cylinder_marker.scale.y = 0.5
        cylinder_marker.scale.z = 1.2  # Cylinder height
        cylinder_marker.color.a = 1.0
        cylinder_marker.color.r = R
        cylinder_marker.color.g = G
        cylinder_marker.color.b = B

        # Create the sphere (head)
        sphere_marker = Marker()
        sphere_marker.header.frame_id = f"human_{human_id}_frame"
        sphere_marker.header.stamp = rospy.Time.now()
        sphere_marker.ns = f"human_markers_{human_id}"
        sphere_marker.id = human_id * 3 + 2  # Unique ID for each human (head)
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = 0
        sphere_marker.pose.position.y = 0
        sphere_marker.pose.position.z = 1.0  # Position above the body
        sphere_marker.scale.x = 0.35  # Sphere diameter
        sphere_marker.scale.y = 0.35
        sphere_marker.scale.z = 0.35
        sphere_marker.color.a = 1.0
        sphere_marker.color.r = R
        sphere_marker.color.g = G
        sphere_marker.color.b = B

        # Create the shoulder markers (positioned above the body at shoulder level)
        shoulder_marker = Marker()
        shoulder_marker.header.frame_id = f"human_{human_id}_frame"
        shoulder_marker.header.stamp = rospy.Time.now()
        shoulder_marker.ns = f"human_markers_{human_id}"
        shoulder_marker.id = human_id * 3 + 3  # Unique ID for each human (shoulder)
        shoulder_marker.type = Marker.SPHERE
        shoulder_marker.action = Marker.ADD
        shoulder_marker.pose.position.x = 0  # Positioned slightly to the right
        shoulder_marker.pose.position.y = 0
        shoulder_marker.pose.position.z = 0.6  # Shoulder level
        shoulder_marker.scale.x = 0.5  # Sphere diameter
        shoulder_marker.scale.y = 0.5
        shoulder_marker.scale.z = 0.5
        shoulder_marker.color.a = 1.0
        shoulder_marker.color.r = R
        shoulder_marker.color.g = G
        shoulder_marker.color.b = B

        # Publish markers
        self.marker_pub.publish(cylinder_marker)
        self.marker_pub.publish(sphere_marker)


if __name__ == '__main__':
    node = ExperimentsNode()
    while not rospy.is_shutdown():
        node.broadcast_dummy()
        node.rate.sleep()