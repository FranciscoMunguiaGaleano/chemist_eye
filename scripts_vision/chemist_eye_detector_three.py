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



BOUNDED_DISTANCE = 8.0
CONFIG_FILE = '../config/camerathree_tf_conf.yaml'
DETECTION_CONFIDENCE = 0.7

def call_chemisteyeone_speech_service():
    # Wait for the service to be available
    rospy.wait_for_service('/run_speech_service_three')  # Make sure the service is available

    try:
        # Create a service proxy to call the service
        run_bash_script = rospy.ServiceProxy('/run_speech_service_three', RunBashScript)
        
        # Call the service and store the response
        response = run_bash_script()

        # Print the result
        if response.success:
            rospy.loginfo("Speech service one executed successfully.")
        else:
            rospy.logwarn(f"Failed to execute speech service three: {response.message}")
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

class YOLOv8Node:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_detection_node_three', anonymous=True)

        # Load YOLOv8 model
        self.model = YOLO("yolov8n.pt", verbose=False)  # YOLOv8 nano model
        self.bridge = CvBridge()
        self.camera_height = 1.5
        self.load_config()

        # Camera intrinsics
        self.fx = 605.6533813476562
        self.fy = 605.4009399414062
        self.cx = 335.85009765625
        self.cy = 244.30010986328125

        # Subscriptions
        self.image_sub = rospy.Subscriber('camerathree/color/image_raw', Image, self.image_callback)
        self.array_sub = rospy.Subscriber('camerathree/depth/array', Float32MultiArray, self.distance_callback)
        self.warning_colour_three_sub = rospy.Subscriber('camerathree/warning_color_topic', String, self.colour_callback)

        # Publisher for annotated images
        self.annotated_image_pub = rospy.Publisher('camerathree/annotated_image', Image, queue_size=10)

        # Publisher for TF transforms
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Publisher for RViz markers
        self.marker_pub = rospy.Publisher('visualization_marker_three', Marker, queue_size=10)

        # Distance tensor
        self.distance_tensor = None

        # Class IDs to detect
        self.target_classes = [0]  # 0 for 'person'

        # Initialize markers and transforms
        self.last_transforms = {}
        
        # Dictionary to track the last time markers were updated
        self.marker_timestamps = {}
        self.counter = 0
        self.last_human_count = 0
        self.markers_colours = "gray"
        #call_chemisteyeone_speech_service()

    def colour_callback(self, msg):
        self.markers_colours = msg.data
        #rospy.loginfo(F"Colour changed to {msg}")
        
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

    def distance_callback(self, msg):
        # Convert the flat list of [x, y, distance] into a NumPy array
        flat_array = np.array(msg.data).reshape(-1, 3)  # Reshape to [N, 3]
    
        # Initialize a tensor with NaN for the entire image resolution
        self.distance_tensor = np.full((480, 640), np.nan, dtype=np.float32)

        # Populate the tensor with the distances
        for entry in flat_array:
            x, y, distance = entry
            self.distance_tensor[int(y), int(x)] = distance


    def get_distance_inside_bbox(self, x1, y1, x2, y2):
        """
        Access the distance in the tensor within the bounding box area and calculate the average
        distance of all points inside the bounding box, filtering out background noise.
        """
        if self.distance_tensor is None:
            return float('nan')  # Return NaN if no distance data

        # Ensure the coordinates are within bounds
        x_min = max(int(x1), 0)
        y_min = max(int(y1), 0)
        x_max = min(int(x2), self.distance_tensor.shape[1] - 1)
        y_max = min(int(y2), self.distance_tensor.shape[0] - 1)

        # Extract the distance values from the bounding box region
        region = self.distance_tensor[y_min:y_max+1, x_min:x_max+1]

        # Filter the valid (non-NaN) distance values
        valid_distances = region[~np.isnan(region)]

        if len(valid_distances) == 0:
            return float('nan')  # Return NaN if no valid distances

        # Fit a Gaussian distribution to the valid distances (person and background separation)
        mu, std = norm.fit(valid_distances)

        # Threshold for background detection (you can adjust this threshold based on empirical data)
        background_threshold = mu + 3 * std  # Consider values more than 2 standard deviations away as background

        # Filter out the background (values that are too far from the mean)
        filtered_distances = valid_distances[valid_distances <= background_threshold]

        if len(filtered_distances) == 0:
            return float('nan')  # If no distances remain after filtering, return NaN

        # Calculate the average of the remaining distances (which should correspond to the person)
        return np.mean(filtered_distances)*0.9


    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLOv8 Prediction (pose estimation)
        results = self.model.predict(source=frame, show=False, task='pose', verbose=False)  # task='pose' for keypoint prediction
        detections = results[0].boxes  # Access bounding boxes

        current_time = rospy.Time.now().to_sec()

        self.counter += 1
        # Annotate image with results
        if self.distance_tensor is not None and detections is not None:
            human_count = 0  # Count detected humans
            detected_humans = []

            for detection in detections:
                xyxy = detection.xyxy[0].tolist()  # Bounding box [x1, y1, x2, y2]
                conf = detection.conf[0]  # Confidence
                cls = int(detection.cls[0])  # Class ID
                label = self.model.names[cls]  # Class name

                # Filter detections for target classes and confidence >= 80%
                if cls in self.target_classes and conf >= DETECTION_CONFIDENCE:
                    # Get the bounding box coordinates
                    x1, y1, x2, y2 = xyxy

                    # Get the distance inside the bounding box
                    distance = self.get_distance_inside_bbox(x1, y1, x2, y2)
                    if distance is not None and distance > 0 and distance < BOUNDED_DISTANCE:
                        # Project to 3D coordinates using the average distance
                        x_center = int((x1 + x2) / 2)
                        y_center = int((y1 + y2) / 2)
                        x_real, y_real, z_real = self.project_to_3d(x_center, y_center, distance)

                        label_text = f"{label} {conf:.2f} ({distance:.2f} m, X={x_real:.2f}, Y={y_real:.2f}, Z={z_real:.2f})"
                        # Broadcast the transform for each person
                        self.broadcast_tf(x_real, y_real, z_real, human_count + 1)

                    else:
                        label_text = f"{label} {conf:.2f} (Distance unavailable)"

                    # Draw the bounding box and labels
                    self.draw_bounding_box(frame, [x1, y1, x2, y2], label_text)

                    # Store the timestamp for the current human's markers
                    detected_humans.append(human_count + 1)
                    self.marker_timestamps[human_count + 1] = current_time

                    # Increment the human counter for the next ID
                    human_count += 1

            # Check and delete markers that are no longer detected
            if human_count < self.last_human_count or human_count == 0: 
                for i in range(1, 10):
                    try:
                        self.delete_markers(i)
                    except:
                        pass
            self.last_human_count = human_count

        # Publish the annotated image
        annotated_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.annotated_image_pub.publish(annotated_image_msg)

    def delete_markers(self, human_id):
        """
        Delete markers for humans that are no longer detected.
        """
        self.counter=0
        delete_marker = Marker()
        delete_marker.header.frame_id = f"human_{human_id}_frame"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = f"human_markers_{human_id}"

        # Set the action to DELETE for each marker
        delete_marker.action = Marker.DELETE
        delete_marker.id = human_id * 3 +1
        self.marker_pub.publish(delete_marker)

        delete_marker.id = human_id * 3 + 2
        self.marker_pub.publish(delete_marker)

        delete_marker.id = human_id * 3 + 3
        self.marker_pub.publish(delete_marker)

        # Remove the human from the timestamp tracker
        del self.marker_timestamps[human_id]

    def get_distance_at(self, x, y):
        """
        Access the distance in the tensor 2D using the (x, y) coordinates and calculates the average
        distance of the 10x10 region around the center.
        """
        if self.distance_tensor is None:
            return float('nan')  # Return NaN if no distance data

        # Get the limits of the 10x10 region around the center
        x_min = max(x - 10, 0)  # Limit to avoid going out of bounds
        y_min = max(y - 10, 0)
        x_max = min(x + 10, self.distance_tensor.shape[1] - 1)
        y_max = min(y + 10, self.distance_tensor.shape[0] - 1)

        # Extract the distance values from the 10x10 region
        region = self.distance_tensor[y_min:y_max+1, x_min:x_max+1]

        # Filter the valid (non-NaN) distance values
        valid_distances = region[~np.isnan(region)]

        # Calculate the average of the valid distances
        if len(valid_distances) > 0:
            return np.mean(valid_distances)
        
        return float('nan')  # Return NaN if no valid distances

    def project_to_3d(self, x, y, z):
        """
        Projects the 2D image coordinates (x, y) and depth (z) into real 3D coordinates.
        """
        x_real = (x - self.cx) * z / self.fx
        y_real = (y - self.cy) * z / self.fy
        z_real = z
        return x_real, y_real, z_real

    def draw_bounding_box(self, frame, xyxy, label):
        # Draw a bounding box and label on the image
        x1, y1, x2, y2 = map(int, xyxy)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green box
        cv2.putText(frame, label, (x1, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

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
        tf_msg.transform.translation.y = - (1.3) / 2 + self.camera_height
        tf_msg.transform.translation.z = z

        # Rotation quaternion for 90 degrees around the X-axis
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
        self.marker_pub.publish(shoulder_marker)

if __name__ == '__main__':
    try:
        node = YOLOv8Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
