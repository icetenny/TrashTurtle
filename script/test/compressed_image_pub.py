#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
import cv2

def publish_compressed_image():
    rospy.init_node('compressed_image_publisher')
    image_pub = rospy.Publisher('/camera/image/compressed', CompressedImage, queue_size=10)

    cap = cv2.VideoCapture(0)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        # Compress the image using OpenCV
        _, img_data = cv2.imencode('.jpg', frame)
        img_data = img_data.tobytes()

        # Create a CompressedImage message
        compressed_msg = CompressedImage()
        compressed_msg.format = 'jpeg'
        compressed_msg.data = img_data

        # Publish the compressed image
        image_pub.publish(compressed_msg)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_compressed_image()
    except rospy.ROSInterruptException:
        pass
