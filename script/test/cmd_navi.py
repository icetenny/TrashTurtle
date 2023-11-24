#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from tf import transformations
import math
from tf.transformations import euler_from_quaternion

class CmdVelNavigator:
    def __init__(self):
        rospy.init_node('cmd_vel_navigator', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_pose = None

        self.lin_v = 0.0
        self.ang_v = 0.0

        self.lin_v_max = 1.5
        self.ang_v_max = 1

        self.lin_a = 0.1
        self.ang_a = 0.1


    
    def quaternion_to_degrees(self, quaternion):
        roll, pitch, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        # Convert angles from radians to degrees
        roll_degrees = math.degrees(roll)
        pitch_degrees = math.degrees(pitch)
        yaw_degrees = math.degrees(yaw)

        return roll_degrees, pitch_degrees, yaw_degrees

 
    def odom_callback(self, msg):
        # Store the current pose from the Odometry message
        self.current_pose = msg.pose.pose

    def navigate_to_point(self, target_x, target_y):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.current_pose is not None:


                # Convert quaternion to degrees
                roll_degrees, pitch_degrees, yaw_degrees = self.quaternion_to_degrees(self.current_pose.orientation)

                if yaw_degrees < 0:
                    yaw_degrees = 360 + yaw_degrees

                # print(f"Roll: {roll_degrees} degrees")
                # print(f"Pitch: {pitch_degrees} degrees")
                print(f"Yaw: {yaw_degrees} degrees")


                # Calculate linear and angular errors
                linear_error = math.sqrt((target_x - self.current_pose.position.x)**2 +
                                         (target_y - self.current_pose.position.y)**2)
                
                goal_angle = math.atan2(target_y - self.current_pose.position.y,target_x - self.current_pose.position.x) * 180 / math.pi

                if goal_angle < 0:
                    goal_angle = 360 + goal_angle
                diff_angle = goal_angle - yaw_degrees
                print("Goal angle", goal_angle)
                print("Diff angle", goal_angle - yaw_degrees)
                
                # Create Twist message
                cmd_vel_msg = Twist()
                # print(angular_error, linear_error, transformations.euler_from_quaternion([
                #                                self.current_pose.orientation.x,
                #                                self.current_pose.orientation.y,
                #                                self.current_pose.orientation.z,
                #                                self.current_pose.orientation.w])[2])

                if abs(diff_angle) > 5 or abs(diff_angle) < 355:
                    self.lin_v = 0
                    if (0 <= diff_angle < 180) or (-180 <= diff_angle < -360):
                        self.ang_v += self.ang_a
                        self.ang_v = min(self.ang_v, self.ang_v_max)
                        cmd_vel_msg.angular.z = self.ang_v  # Adjust the angular speed as needed
                    else:
                        self.ang_v -= self.ang_a
                        self.ang_v = max(self.ang_v, -self.ang_v_max)
                        cmd_vel_msg.angular.z = self.ang_v
                else:
                    self.ang_v = 0

                    self.lin_v += self.lin_a
                    self.lin_v = min(self.lin_v, self.lin_v_max)

                    cmd_vel_msg.linear.x = self.lin_v  # Adjust the linear speed as needed
                # Publish Twist message
                self.cmd_vel_pub.publish(cmd_vel_msg)

                # Check if the target is reached
                if linear_error < 0.05:
                    rospy.loginfo("Target reached!")
                    break

            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = CmdVelNavigator()
        while True:
            navigator.navigate_to_point(0, 0)
            navigator.navigate_to_point(1.15, 0.1)
            navigator.navigate_to_point(1.27, -1.03)  # Adjust the target coordinates as needed
            navigator.navigate_to_point(0.1, -1.14)
    except rospy.ROSInterruptException:
        pass
