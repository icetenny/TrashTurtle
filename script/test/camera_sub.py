#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

def image_callback(msg):
    # Convert ROS Image message to NumPy array
    img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

    # Display the image using OpenCV
    img = cv2.rotate(img_np, cv2.ROTATE_180)
    cv2.imshow("Image", img)
    cv2.waitKey(1)  # Adjust the waitKey value to control the display speed

def image_listener():
    rospy.init_node('image_listener', anonymous=True)

    # Replace 'your_image_topic' with the actual topic name where your image sensor data is published
    rospy.Subscriber('/camera_frame', Image, image_callback)

    rospy.spin()

if __name__ == '__main__':
    image_listener()
    cv2.destroyAllWindows()
