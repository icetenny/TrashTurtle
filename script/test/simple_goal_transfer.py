#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def transfer_callback(data):
    goal_pub.publish(data)

def main():
    global goal_pub
    rospy.init_node('simple_goal_transfer')

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('/simple_goal', PoseStamped, transfer_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
