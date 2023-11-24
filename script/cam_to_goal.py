#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Bool, String, Float32, Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def ok_callback(msg):
    global goal_pub, current_pose, dis, ang
    x, y = current_pose.position.x, current_pose.position.y
    rz, rw = current_pose.orientation.z, current_pose.orientation.w

    transformed_pose = transform_pose((x,y,rz,rw), distance=dis, angle=ang)

    print(x,y, transformed_pose[0], transformed_pose[1])
    move_to_goal(transformed_pose)
            
def transform_pose(current_pose, distance=0.5, angle=0):
    x, y, rz, rw = current_pose
    roll, pitch, yaw = euler_from_quaternion([0,0,rz, rw])
    yaw += math.radians(angle)
    new_quaternion = quaternion_from_euler(roll, pitch, yaw)

    goal_x = x + distance * math.cos(yaw)
    goal_y = y + distance * math.sin(yaw)

    return (goal_x, goal_y, new_quaternion[2], new_quaternion[3])

def pose_callback(msg:PoseWithCovarianceStamped):
    global current_pose
    current_pose = msg.pose.pose

def bounding_box_callback(msg:String):
    global dis, ang
    x1, y1, x2, y2 = [float(i) for i in msg.data.split()]

    dis = map_value(y2, [0.5, 1], [1, 0.23]) - 0.18
    ang = map_value((x1+x2)/2, [0, 1], [40, -40])

    print(dis, ang)

def map_value(value, from_range, to_range):
    normalized_position = (value - from_range[0]) / (from_range[1] - from_range[0])
    mapped_value = to_range[0] + normalized_position * (to_range[1] - to_range[0])

    return mapped_value


    
# def initial_callback(msg:PoseWithCovarianceStamped):
#     global goal_pub

#     current_pose = msg.pose.pose

#     x, y = current_pose.position.x, current_pose.position.y
#     rz, rw = current_pose.orientation.z, current_pose.orientation.w

#     transformed_pose = transform_pose((x,y,rz,rw), distance=0.5, angle=-45)

#     print(x,y, transformed_pose[0], transformed_pose[1])
#     move_to_goal(transformed_pose)

def move_to_goal(goal):
    global goal_pub
    x, y, rz, rw = goal
    pub_goal = PoseStamped()
    pub_goal.header.stamp = rospy.Time.now()
    pub_goal.header.frame_id = "map"

    pub_goal.pose.position.x = x
    pub_goal.pose.position.y = y

    pub_goal.pose.orientation.z = rz
    pub_goal.pose.orientation.w = rw

    # rospy.loginfo(f"Sending goal point: ({goal})")
    for i in range(5):
        goal_pub.publish(pub_goal)


def main():
    global goal_pub, current_pose

    rospy.init_node("Cam_to_goal", anonymous=True)

    rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    
    # rospy.Subscriber(
    #         '/initialpose', PoseWithCovarianceStamped, initial_callback)
    rospy.Subscriber(
            '/ok', Float32, ok_callback)
    
    rospy.Subscriber(
            '/bounding_box', String, bounding_box_callback)
    
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

