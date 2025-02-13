#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

MAX_TEMPERATURE = 200  # Maximum threshold for fire detection

def get_color_from_temperature(temperature):
    """ Returns an RGB color transitioning from blue (cold) to red (hot). """
    normalized_temp = min(1.0, max(0.0, temperature / MAX_TEMPERATURE))  # Clamp between 0 and 1
    r = normalized_temp  # More red as it gets hotter
    g = 0.0  # Green removed for a cleaner transition
    b = 1.0 - normalized_temp  # Blue fades out
    return r, g, b

def create_sphere_marker(temp, frame_id, marker_id):
    """ Creates an RViz sphere marker based on temperature. """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "fire_detection"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.scale.x = 0.9  # Sphere size
    marker.scale.y = 0.9
    marker.scale.z = 0.9

    r, g, b = get_color_from_temperature(temp)  # Get color from temperature
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 0.6  # Transparency for better visibility

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.2  # Sphere's height

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    return marker

def create_text_marker(temp, frame_id, marker_id):
    """ Creates a text marker displaying the temperature above the sphere. """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "fire_detection_text"
    marker.id = marker_id + 100  # Unique ID offset for text
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    marker.scale.z = 0.4  # Text size

    marker.color.r = 1.0  # White text
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0  # Fully visible

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.8  # Slightly above the sphere

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.text = f"{temp:.1f}°C"  # Display temperature

    return marker

def temperature_callback(msg, args):
    """ Callback for temperature sensors, publishing markers in RViz. """
    frame_id, marker_id, marker_pub, text_pub = args
    temperature = msg.data
    
    if temperature > MAX_TEMPERATURE:
        rospy.logwarn(f"🔥 Potential fire detected! Temperature exceeded {MAX_TEMPERATURE}°C in {frame_id}!")

    # Publish sphere marker
    sphere_marker = create_sphere_marker(temperature, frame_id, marker_id)
    marker_pub.publish(sphere_marker)

    # Publish temperature text marker
    text_marker = create_text_marker(temperature, frame_id, marker_id)
    text_pub.publish(text_marker)

def fire_detection_node():
    """ Main node for fire detection with RViz visualization. """
    rospy.init_node('fire_detection_node', anonymous=True)
    
    # Publishers for RViz markers
    marker_pub1 = rospy.Publisher('fire_marker_1', Marker, queue_size=10)
    text_pub1 = rospy.Publisher('fire_marker_text_1', Marker, queue_size=10)

    marker_pub2 = rospy.Publisher('fire_marker_2', Marker, queue_size=10)
    text_pub2 = rospy.Publisher('fire_marker_text_2', Marker, queue_size=10)

    # Subscribers to temperature topics
    rospy.Subscriber('camerair/max_temperature', Float32, temperature_callback, ('IR_camera_link', 1, marker_pub1, text_pub1))
    rospy.Subscriber('camerair_two/max_temperature', Float32, temperature_callback, ('IR_cameratwo_link', 2, marker_pub2, text_pub2))

    rospy.loginfo("🔥 Fire detection node started. Monitoring temperature with RViz markers...")
    rospy.spin()

if __name__ == '__main__':
    try:
        fire_detection_node()
    except rospy.ROSInterruptException:
        pass
