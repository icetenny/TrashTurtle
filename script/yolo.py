#!/usr/bin/env python3

import cv2
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32, Header
import numpy as np
import sys
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TrashYOLO():
    def __init__(self):
        self.model = YOLO("/home/ice/catkin_ws/src/final/weights/best.pt")
        rospy.init_node('detector', anonymous=True)

        self.command_pub = rospy.Publisher('/command', String, queue_size=10)
        self.box_pub = rospy.Publisher('/bounding_box', String, queue_size=10)
        self.trash_pose_pub = rospy.Publisher('/trash_pose', String, queue_size=10)
        self.frame_web_pub = rospy.Publisher('/frame_web', CompressedImage, queue_size=10)

        rospy.Subscriber('/camera_frame', CompressedImage, self.image_callback, queue_size=3)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/active_state', String, self.state_callback)

        self.trash_dis, self.trash_ang = 0, 0
        self.current_pose = None
        self.actvate_state = None
        self.img = None

        self.frame_count = 0
        self.frame_interval = 15

    def cv2_to_ros_image(self, frame):
        _, img_data = cv2.imencode('.jpg', frame)
        img_data = img_data.tobytes()

        # Create a CompressedImage message
        compressed_msg = CompressedImage()
        compressed_msg.format = 'jpeg'
        compressed_msg.data = img_data

        # Publish the compressed image
        return compressed_msg


    def transform_pose(self):
        x, y, rz, rw = self.current_pose
        roll, pitch, yaw = euler_from_quaternion([0,0,rz, rw])
        yaw += math.radians(self.trash_ang)
        new_quaternion = quaternion_from_euler(roll, pitch, yaw)

        goal_x = x + self.trash_dis * math.cos(yaw)
        goal_y = y + self.trash_dis * math.sin(yaw)

        return (goal_x, goal_y, new_quaternion[2], new_quaternion[3])

    def map_value(self, value, from_range, to_range):
        normalized_position = (value - from_range[0]) / (from_range[1] - from_range[0])
        mapped_value = to_range[0] + normalized_position * (to_range[1] - to_range[0])

        return mapped_value

    def pose_callback(self, msg:PoseWithCovarianceStamped):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    def state_callback(self, msg:String):
        self.actvate_state = msg.data

    def image_callback(self, msg):
        print("Image Received.. YOLOing...")
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # img 
        self.img = cv2.rotate(image_np, cv2.ROTATE_180)

        self.frame_count = (self.frame_count + 1) % self.frame_interval
        if self.frame_count == 0:
            self.frame_web_pub.publish(msg)
            print("Pub to Web")

def main():
    yolo = TrashYOLO()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        results = yolo.model.predict(yolo.img, verbose=False, conf=0.7)[0]

        coor = ""

        if len(results.boxes) > 0:
            bb = results.boxes[0]
            # Extract coordinates from the bounding box
            bb_int = [int(coord) for coord in bb.xyxy[0].cpu().numpy()]
            # Draw bounding box on the image
            cv2.rectangle(yolo.img, (bb_int[0], bb_int[1]), (bb_int[2], bb_int[3]), (0, 255, 0), 2)

            xyxyn = results.boxes.xyxyn.tolist()[0]

            x1, y1, x2, y2 = xyxyn

            if y2 > 0.5 and yolo.current_pose:
                coor = " ".join((str(i) for i in xyxyn))
                yolo.trash_dis = yolo.map_value(y2, [0.5, 1], [1, 0.23]) - 0.18

                yolo.trash_ang = yolo.map_value((x1+x2)/2, [0, 1], [22, -22])

                transform_trash_pose = yolo.transform_pose()

                print(transform_trash_pose)
                yolo.trash_pose_pub.publish(" ".join((str(i) for i in transform_trash_pose)))

                if yolo.actvate_state == "STATE_Walking":
                    yolo.command_pub.publish('targeting')

        yolo.box_pub.publish(coor)
        
        cv2.imshow("f", yolo.img)
        key = cv2.waitKey(1)

        if key == ord('q'):
            print("EXIT")
            cv2.destroyAllWindows()
            sys.exit(0)

        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    main()

    