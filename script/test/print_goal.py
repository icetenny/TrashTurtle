#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def goal_callback(msg):
    x = msg.pose.position.x
    y = msg.pose.position.y
    rz = msg.pose.orientation.z
    rw = msg.pose.orientation.w

    print((x,y,rz,rw))

def main():
    rospy.init_node('goal_subscriber', anonymous=True)

    # Subscribe to the move_base_simple/goal topic
    rospy.Subscriber('move_base_simple/goal', PoseStamped, goal_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
