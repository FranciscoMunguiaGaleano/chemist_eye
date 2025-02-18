#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped
import math

def broadcast_transforms():
    rospy.init_node('kuka_mobile_base_tf_broadcaster')
    ns = rospy.get_param('~ns_name', "robot1")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)

    t = 0

    while not rospy.is_shutdown():
        # Broadcast transform for kuka_mobile_base_link
        br.sendTransform((0.0, 0.0, 0.17),  # Origin of kuka_mobile_base_link relative to base_plate_1
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         ns + "_base_plate_1",          # Child frame (base plate 2)
                         ns + "_kuka_mobile_base_link") # Parent frame (base plate 1)

        # Broadcast transform for base_plate_2
        br.sendTransform((0.0, 0.0, 0.055),  # Origin of base_plate_2 relative to base_plate_1
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         ns + "_base_plate_2",  # Child frame (base plate 2)
                         ns + "_base_plate_1")  # Parent frame (base plate 1)

        # Broadcast transform for base_block
        br.sendTransform((0.0, 0.0, 0.29),   # Origin of base_block relative to base_plate_2
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         ns + "_base_block",    # Child frame (base block)
                         ns + "_base_plate_2")  # Parent frame (base plate 2)

        # Broadcast transform for top_cylinder
        br.sendTransform((-0.37, 0.19, 0.39), # Origin of top_cylinder relative to base_block
                         tf.transformations.quaternion_from_euler(0, 0, math.pi),
                         rospy.Time.now(),
                         ns + "_top_cylinder",  # Child frame (top cylinder)
                         ns + "_base_block")    # Parent frame (base block)
        
        # Broadcast transform for kuka_mobile_base_top
        br.sendTransform((0.0, 0.0, 0.13),    # Origin of kuka_mobile_base_top relative to top_cylinder
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         ns + "_kuka_mobile_base_top",   # Child frame (mobile base top)
                         ns + "_top_cylinder")           # Parent frame (top cylinder)
        # Broadcast transform for kuka_mobile_base_top
        br.sendTransform((0.0, 0.0, 0.0),    # Origin of kuka_mobile_base_top relative to top_cylinder
                         tf.transformations.quaternion_from_euler(0, 0, -math.pi/2),
                         rospy.Time.now(),
                         ns + "_base",   # Child frame (mobile base top)
                         ns + "_kuka_mobile_base_top")           # Parent frame (top cylinder)
        # Broadcast transform for kuka_mobile_base_top
        br.sendTransform((0.0, 0.0, 0.0),    # Origin of kuka_mobile_base_top relative to top_cylinder
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         ns + "_base_link",   # Child frame (mobile base top)
                         ns + "_base")           # Parent frame (top cylinder)
        # KUKA arm links
        br.sendTransform((0.0, 0.0, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0, -math.pi/2),
                         rospy.Time.now(),
                         ns + "_link_1",
                         ns + "_base_link")

        br.sendTransform((-0.00043624, 0.0, 0.36),
                         tf.transformations.quaternion_from_euler(0, math.pi/10, 0),
                         rospy.Time.now(),
                         ns + "_link_2",
                         ns + "_link_1")

        br.sendTransform((0.0, 0.0, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0.0, 0),
                         rospy.Time.now(),
                         ns + "_link_3",
                         ns + "_link_2")

        br.sendTransform((0.00043624, 0.0, 0.42),
                         tf.transformations.quaternion_from_euler(0, math.pi/2 + math.pi/4, 0),
                         rospy.Time.now(),
                         ns + "_link_4",
                         ns + "_link_3")

        br.sendTransform((0.0, 0.0, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         ns + "_link_5",
                         ns + "_link_4")

        br.sendTransform((0.0, 0.0, 0.4),
                         tf.transformations.quaternion_from_euler(0, math.pi/7, 0),
                         rospy.Time.now(),
                         ns + "_link_6",
                         ns + "_link_5")

        br.sendTransform((0.0, 0.0, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         ns + "_link_7",
                         ns + "_link_6")

        # Tool frame (end effector)
        br.sendTransform((0.0, 0.0, 0.126),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         ns + "_tool0",
                         ns + "_link_7")

        rate.sleep()

if __name__ == '__main__':
    broadcast_transforms()
