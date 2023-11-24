#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Float32


class TrashTopic:
    def __init__(self, topic, message_type):
        self.message_received = False
        self.subscriber = rospy.Subscriber(topic, message_type, self.callback)
        self.publisher = rospy.Publisher(topic, message_type, queue_size=10)
        self.rate = rospy.Rate(10)

    def callback(self, data):
        rospy.loginfo("Received servo angle: %f", data.data)
        self.message_received = True

    def pub_and_check(self, data):
        while not self.message_received:
            self.publisher.publish(data)
            self.rate.sleep()


def main():
    rospy.init_node('servo_test', anonymous=True)

    servo = TrashTopic("/servo1/servo_angle", Float32)

    servo.pub_and_check(120)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
