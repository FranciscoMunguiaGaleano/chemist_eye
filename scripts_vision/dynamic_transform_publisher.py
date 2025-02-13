#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import tf
from std_msgs.msg import Float32

# Define a class to publish dynamic transforms
class DynamicTransformPublisher:
    def __init__(self):
        rospy.init_node('dynamic_transform_publisher')

        # Create a TF2 broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Set up publishers for the slider values (translation and rotation)
        self.translation_x_pub = rospy.Publisher('/translation_x', Float32, queue_size=10)
        self.translation_y_pub = rospy.Publisher('/translation_y', Float32, queue_size=10)
        self.translation_z_pub = rospy.Publisher('/translation_z', Float32, queue_size=10)
        self.rotation_x_pub = rospy.Publisher('/rotation_x', Float32, queue_size=10)
        self.rotation_y_pub = rospy.Publisher('/rotation_y', Float32, queue_size=10)
        self.rotation_z_pub = rospy.Publisher('/rotation_z', Float32, queue_size=10)

        # Set initial values for the transform
        self.translation_x = 3.0
        self.translation_y = 0.0
        self.translation_z = 1.5
        self.rotation_x = 0.0
        self.rotation_y = 0.0
        self.rotation_z = 0.0

        # Create and start the loop that will publish the transform continuously
        self.publish_transform()

    def publish_transform(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Convert Euler angles (roll, pitch, yaw) to quaternion
            quaternion = tf.transformations.quaternion_from_euler(
                self.rotation_x, self.rotation_y, self.rotation_z
            )

            # Create a TransformStamped message
            t = geometry_msgs.msg.TransformStamped()
            
            # Set the header
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "camera_link"
            
            # Set the translation from the sliders (initial values are set above)
            t.transform.translation.x = self.translation_x
            t.transform.translation.y = self.translation_y
            t.transform.translation.z = self.translation_z

            # Set the rotation as the quaternion
            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            # Publish the transform
            self.broadcaster.sendTransform(t)

            # Sleep for a bit
            rate.sleep()

    # Methods to update translation and rotation values (to be triggered by slider changes)
    def update_translation_x(self, val):
        self.translation_x = val.data

    def update_translation_y(self, val):
        self.translation_y = val.data

    def update_translation_z(self, val):
        self.translation_z = val.data

    def update_rotation_x(self, val):
        self.rotation_x = val.data

    def update_rotation_y(self, val):
        self.rotation_y = val.data

    def update_rotation_z(self, val):
        self.rotation_z = val.data


if __name__ == "__main__":
    # Create the dynamic transform publisher object
    dynamic_transform_publisher = DynamicTransformPublisher()

    # Spin to keep the node running
    rospy.spin()

