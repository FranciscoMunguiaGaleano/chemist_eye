#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageJoiner:
    def __init__(self):
        rospy.init_node("image_joiner", anonymous=True)
        self.bridge = CvBridge()

        # Image placeholders
        self.annotated_1 = None
        self.annotated_2 = None
        self.annotated_3 = None
        self.ir_1 = None
        self.ir_2 = None

        # Subscribing to image topics with dedicated callbacks
        rospy.Subscriber("/camera/annotated_image", Image, self.image_annotated_1_callback)
        rospy.Subscriber("/camerathree/annotated_image", Image, self.image_annotated_2_callback)
        rospy.Subscriber("/cameratwo/annotated_image", Image, self.image_annotated_3_callback)
        rospy.Subscriber("/camerair/color/image_raw", Image, self.image_ir_1_callback)
        rospy.Subscriber("/camerair_two/color/image_raw", Image, self.image_ir_2_callback)

        # Publisher for combined image
        self.combined_image_pub = rospy.Publisher("/joined_image", Image, queue_size=1)

        rospy.loginfo("📸 Image Joiner Node Initialized!")

    def image_annotated_1_callback(self, msg):
        """Handles incoming Annotated Image 1"""
        self.annotated_1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def image_annotated_2_callback(self, msg):
        """Handles incoming Annotated Image 2"""
        self.annotated_2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def image_annotated_3_callback(self, msg):
        """Handles incoming Annotated Image 3"""
        self.annotated_3 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def image_ir_1_callback(self, msg):
        """Handles incoming IR Image 1"""
        self.ir_1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def image_ir_2_callback(self, msg):
        """Handles incoming IR Image 2"""
        self.ir_2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def create_canvas(self, canvas_height, canvas_width):
        """Create a black canvas for the final image"""

        # Create a blank black canvas
        canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

        return canvas

    def join_images(self):
        """Join the available images into one large image and show it with OpenCV"""

        # Create a blank canvas (height 960px, width 1280px)
        canvas = self.create_canvas(960, 1280)

        # Top row: Annotated 1 and Annotated 2
        if self.annotated_1 is not None:
            canvas[:480, :640] = self.annotated_1  # Top-left (Annotated 1)
        else:
            canvas[:480, :640] = np.zeros((480, 640, 3), dtype=np.uint8)  # Black space for missing Annotated 1

        if self.annotated_2 is not None:
            canvas[:480, 640:1280] = self.annotated_2  # Top-right (Annotated 2)
        else:
            canvas[:480, 640:1280] = np.zeros((480, 640, 3), dtype=np.uint8)  # Black space for missing Annotated 2

        # Bottom row: Annotated 3, gap, IR 1, IR 2
        if self.annotated_3 is not None:
            canvas[480:, :640] = self.annotated_3  # Bottom-left (Annotated 3)
        else:
            canvas[480:, :640] = np.zeros((480, 640, 3), dtype=np.uint8)  # Black space for missing Annotated 3

        # Add a gap between Annotated 3 and IR 1 (if Annotated 3 is present)
        if self.annotated_3 is not None:
            canvas[480:, 640:640+20] = np.zeros((480, 20, 3), dtype=np.uint8)  # Black gap (20px)
        
        # Resize IR images to match canvas height (480px)
        if self.ir_1 is not None:
            ir_1_resized = cv2.resize(self.ir_1, (310, 480))  # Resize IR 1 to 310x480
            canvas[480:, 640+20:640+20+310] = ir_1_resized  # Bottom-center-left (IR 1)
        else:
            canvas[480:, 640+20:640+20+310] = np.zeros((480, 310, 3), dtype=np.uint8)  # Black space for missing IR 1

        if self.ir_2 is not None:
            ir_2_resized = cv2.resize(self.ir_2, (310, 480))  # Resize IR 2 to 310x480
            canvas[480:, 640+20+310:640+20+310+310] = ir_2_resized  # Bottom-center-right (IR 2)
        else:
            canvas[480:, 640+20+310:640+20+310+310] = np.zeros((480, 310, 3), dtype=np.uint8)  # Black space for missing IR 2

        # Visualize the image using OpenCV
        cv2.imshow("Combined Image", canvas)
        cv2.waitKey(1)  # Wait for a short time (1 ms) to update the window

        # Convert the final image to ROS Image message and publish
        try:
            msg = self.bridge.cv2_to_imgmsg(canvas, "bgr8")
            self.combined_image_pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr("Error converting image: %s", e)



    def run(self):
        """Run the node and keep it alive."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.join_images()  # Attempt to join and publish images
            rate.sleep()

if __name__ == "__main__":
    node = ImageJoiner()
    node.run()
