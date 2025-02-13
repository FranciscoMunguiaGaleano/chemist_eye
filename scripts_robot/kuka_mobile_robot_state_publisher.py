#!/usr/bin/env python
import rospy
import tf
import math

def move_base():
    rospy.init_node('mobile_base_tf_broadcaster')

    # Get parameters from launch file (default values if not specified)
    x_init = rospy.get_param('~x_init', -2.5)
    y_init = rospy.get_param('~y_init', 2.5)
    yaw_init = rospy.get_param('~yaw_init', 0.0)
    ns = rospy.get_param('~ns_name', "robot1")

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    t = 0

    while not rospy.is_shutdown():
        # Use the parameters to modify the robot's position and orientation
        x = x_init + 1.0 * math.cos(t)
        y = y_init + 1.0 * math.sin(t)
        yaw = yaw_init + t
        x = x_init
        y = y_init

        br.sendTransform((x, y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, yaw),
                         rospy.Time.now(),
                         ns + "_kuka_mobile_base_link",
                         "world")

        t += 0.1
        rate.sleep()

if __name__ == '__main__':
    move_base()

