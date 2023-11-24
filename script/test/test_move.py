#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatusArray
from itertools import cycle


class TrashTopic:
    def __init__(self, topic, message_type):
        self.message_received = False
        self.subscriber = rospy.Subscriber(topic, message_type, self.callback)
        self.publisher = rospy.Publisher(topic, message_type, queue_size=10)
        self.rate = rospy.Rate(10)

    def callback(self, data):
        print("Receive")
        self.message_received = True

    def pub_and_check(self, data):
        # while not self.message_received:
        #     self.publisher.publish(data)
        #     self.rate.sleep()

        for i in range(10):
            self.publisher.publish(data)
            self.rate.sleep()


def move_to_goal(x, y, rz, rw):
    global goal_topic
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = x
    goal.pose.position.y = y

    goal.pose.orientation.z = rz
    goal.pose.orientation.w = rw

    rospy.loginfo("Sending goal point: (%f, %f)", x, y)
    goal_topic.pub_and_check(goal)

def goal_status_callback(msg):
    # Check if any goal is active
    if msg.status_list:
        goal_status = msg.status_list[0].status
        if goal_status == 3:  # Goal reached (SUCCEEDED)
            # Publish the next goal
            publish_next_goal()

def publish_next_goal():
    global goal_iterator
    next_goal = next(goal_iterator)
    move_to_goal(*next_goal)


def main():
    global goal_topic
    global goal_iterator

    rospy.init_node('autonomous_navigation', anonymous=True)

    # Set the goal points
    goals = [
        (0.676, 1.263, -0.861, 0.5085),
        (0.211, 0.595, -0.214, 0.976),
        (1.085, -0.113, 0.458, 0.888),
        (1.577, 0.60, 0.9566, 0.291)
    ]

    goal_iterator = cycle(goals)

    # Subscribe to the move_base/result topic to get goal status feedback
    # rospy.Subscriber('/move_base/status', GoalStatusArray, goal_status_callback)
    # rospy.Subscriber('/move_base/command', GoalStatusArray, command_callback)

    goal_topic = TrashTopic('/move_base_simple/goal', PoseStamped)

    # Publish the first goal
    publish_next_goal()

    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
