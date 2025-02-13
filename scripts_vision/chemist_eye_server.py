#!/usr/bin/env python
from flask import Flask, request, jsonify
import base64
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32
from cv_bridge import CvBridge

# Initialize ROS 1 node
rospy.init_node('flask_realsense_publisher', anonymous=True)

# Define ROS publishers for existing cameras
image_pub = rospy.Publisher('camera/color/image_raw', Image, queue_size=10)
array_pub = rospy.Publisher('camera/depth/array', Float32MultiArray, queue_size=10)
imagetwo_pub = rospy.Publisher('cameratwo/color/image_raw', Image, queue_size=10)
arraytwo_pub = rospy.Publisher('cameratwo/depth/array', Float32MultiArray, queue_size=10)
imagethree_pub = rospy.Publisher('camerathree/color/image_raw', Image, queue_size=10)
arraythree_pub = rospy.Publisher('camerathree/depth/array', Float32MultiArray, queue_size=10)

# Define ROS publishers for the first IR camera
imageir_pub = rospy.Publisher('camerair/color/image_raw', Image, queue_size=10)
tempir_pub = rospy.Publisher('camerair/max_temperature', Float32, queue_size=10)

# Define ROS publishers for the second IR camera
imageirtwo_pub = rospy.Publisher('camerair_two/color/image_raw', Image, queue_size=10)
tempirtwo_pub = rospy.Publisher('camerair_two/max_temperature', Float32, queue_size=10)

# Initialize OpenCV bridge
bridge = CvBridge()

# Flask app
app = Flask(__name__)

@app.route('/upload', methods=['POST'])
def upload():
    try:
        data = request.get_json()
        image_base64 = data['image']
        distance_array = np.array(data['distance_array'])

        # Decode image
        image_data = base64.b64decode(image_base64)
        nparr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # Publish image and depth array
        image_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        array_msg = Float32MultiArray(data=distance_array.flatten().tolist())
        array_pub.publish(array_msg)

        return jsonify({'status': 'success', 'message': 'Data received and published!'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/uploadtwo', methods=['POST'])
def uploadtwo():
    try:
        data = request.get_json()
        image_base64 = data['image']
        distance_array = np.array(data['distance_array'])

        # Decode image
        image_data = base64.b64decode(image_base64)
        nparr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # Publish image and depth array
        imagetwo_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        array_msg = Float32MultiArray(data=distance_array.flatten().tolist())
        arraytwo_pub.publish(array_msg)

        return jsonify({'status': 'success', 'message': 'Data received and published!'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/uploadthree', methods=['POST'])
def uploadthree():
    try:
        data = request.get_json()
        image_base64 = data['image']
        distance_array = np.array(data['distance_array'])

        # Decode image
        image_data = base64.b64decode(image_base64)
        nparr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # Publish image and depth array
        imagethree_pub.publish(bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        array_msg = Float32MultiArray(data=distance_array.flatten().tolist())
        arraythree_pub.publish(array_msg)

        return jsonify({'status': 'success', 'message': 'Data received and published!'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/upload_ir', methods=['POST'])
def upload_ir():
    try:
        data = request.get_json()
        image_base64 = data['image']
        max_temp = float(data['max_temperature'])

        # Decode IR image
        image_data = base64.b64decode(image_base64)
        nparr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)  

        # Publish IR image
        image_msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        imageir_pub.publish(image_msg)

        # Publish max temperature
        temp_msg = Float32(data=max_temp)
        tempir_pub.publish(temp_msg)

        return jsonify({'status': 'success', 'message': 'IR data from ir_camera 1 received and published!'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/upload_ir_two', methods=['POST'])
def upload_ir_two():
    try:
        data = request.get_json()
        image_base64 = data['image']
        max_temp = float(data['max_temperature'])

        # Decode IR image
        image_data = base64.b64decode(image_base64)
        nparr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)  

        # Publish IR image
        image_msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        imageirtwo_pub.publish(image_msg)

        # Publish max temperature
        temp_msg = Float32(data=max_temp)
        tempirtwo_pub.publish(temp_msg)

        return jsonify({'status': 'success', 'message': 'IR data from ir_camera 2 received and published!'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 400

if __name__ == '__main__':
    try:
        app.run(host='192.168.1.101', port=5000)
    except rospy.ROSInterruptException:
        pass


