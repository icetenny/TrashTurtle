#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

def process_compressed_image(msg):
    # Decompress the image using OpenCV
    np_arr = np.frombuffer(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Process the decompressed image (replace this with your processing logic)
    # For example, you can display the image using OpenCV
    cv2.imshow('Decompressed Image', image_np)
    cv2.waitKey(1)

def subscribe_to_compressed_image():
    rospy.init_node('compressed_image_subscriber')
    rospy.Subscriber('/camera_frame', CompressedImage, process_compressed_image)
    rospy.spin()

if __name__ == '__main__':
    subscribe_to_compressed_image()
