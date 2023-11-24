#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from std_msgs.msg import Int32, Bool, String, Float32, Header
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray, GoalID
from itertools import cycle
import math


class TrashTurtle():
    def __init__(self) -> None:
        self.sm = smach.StateMachine(outcomes=['end'])

        self.output_to_state = {'sleeping': 'STATE_Sleeping', 'go_home': 'STATE_GoHome', 'walking': 'STATE_Walking',
                                'targeting': 'STATE_Targeting', 'picking': 'STATE_Picking', 'go_trash': 'STATE_GoTrash', 'placing': 'STATE_Placing'}

        # Variable #################################
        self.active_state = self.sm.get_active_states()[0]
        self.command = ''
        self.goal_reached = False
        self.detect_bounding_box = []
        self.cmd_vel = Twist()
        self.trash_pose = []
        self.last_goal = None
        self.amcl_pose = None


        # Constant #################################
        # MI
        # self.home_position = (-0.5, -0.4566, 0.1986, 0.98)
        # self.trash_position = (2.04, -0.65, -0.585, 0.8112)

        # self.goals = [
        #     (0.62, -0.34, -0.546, 0.837),
        #     (1.939, 0.17, 0.1989, 0.98),
        #     (1.58, 1.13, 0.8406, 0.5417),
        #     (0.46, 0.72, -0.9866, 0.1627)
        # ]

        # self.home_position = (1.0599998235702515, -1.1100001335144043, 0.8822710977194863, 0.47074165964874254)

        # self.trash_position = (-1.040000081062317, -0.6699997782707214, 0.867433997011303, 0.49755226944412057)

        # self.goals = [
        #     (0.12000001966953278, -0.6999998688697815, 0.8683273874905283, 0.4959914799004857),
        #     (-0.6100003719329834, -0.8600001931190491, -0.9851890046578851, 0.17147193677452227),
        #     (-0.2500004172325134, -1.5800000429153442, -0.39854834539422096, 0.9171473253428419),
        #     (0.7099998593330383, -1.2699999809265137, 0.2162795556262078, 0.9763314774287113)
        #     ]

        # Real

        self.home_position = (0.07999999076128006, -0.35000017285346985, 0.02937379358416798, 0.9995684970278299)
        self.trash_position = (2.5099995136260986, -1.28000009059906, -0.6881294080555037, 0.7255879807226564)

        self.goals = [
            (1.0699996948242188, -0.8900002241134644, -0.007632980507741884, 0.9999708683799585),
            (3.109999656677246, -0.5000003576278687, 0.7196149913243335, 0.6943732888448976),
            (2.7099995613098145, 0.6399995684623718, -0.9999829025506038, 0.005847615451569677),
            (1.040000081062317, 0.6599997282028198, -0.6990885438738675, 0.7150351095046421)
        ]
        self.goal_iterator = cycle(self.goals)

        # Publisher ###############################
        self.servo1_pub = rospy.Publisher(
            '/servo1/servo_angle', Float32, queue_size=10)
        self.servo2_pub = rospy.Publisher(
            '/servo2/servo_angle', Float32, queue_size=10)
        self.simple_goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cancel_pub = rospy.Publisher(
            '/move_base/cancel', GoalID, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.state_pub = rospy.Publisher('/active_state', String, queue_size=10)

        self.state_pub.publish('STATE_Sleeping')

        # Subscriber #############################
        rospy.Subscriber('/command', String, self.command_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray,
                         self.move_base_status_callback)
        rospy.Subscriber('/bounding_box', String, self.bounding_box_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)
        rospy.Subscriber('/trash_pose', String, self.trash_pose_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        # rospy.Subscriber(
        #         '/bounding_box', String, bounding_box_callback)


        # Start up #############################################3
        self.servo_command(1, 'open')
        self.servo_command(2, 'open')
        self.servo_command(2, 'close')
        self.servo_command(1, 'close')


    def add_state(self, name: str, state: smach.State, outcomes: list):
        trans = dict()
        for out in outcomes:
            trans[out] = self.output_to_state[out]

        smach.StateMachine.add(name, state(
            self, name, outcomes), transitions=trans)
        
    def pub_activate_state(self):
        self.state_pub.publish(self.active_state)

    def enter_new_state(self):
        self.active_state = self.sm.get_active_states()[0]
        self.command = ''
        self.goal_reached = False
        # self.new_goal_assigned = False
        self.trash_pose = []
        self.pub_activate_state()


        rospy.loginfo(self.active_state)

    def command_callback(self, msg:String):
        self.command = msg.data

    def bounding_box_callback(self, msg:String):
        self.detect_bounding_box = [float(i) for i in msg.data.split()]

    def vel_callback(self, msg:Twist):
        self.cmd_vel = msg

    def trash_pose_callback(self, msg:String):
        self.trash_pose = [float(i) for i in msg.data.split()]

    def amcl_callback(self, msg:PoseWithCovarianceStamped):
        self.amcl_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]


    def is_stop(self):
        return self.cmd_vel.linear.x == 0 and self.cmd_vel.angular.z == 0
    
    def at_goal(self, tolerance = 0.2):
        if self.last_goal:
            pose1 = self.last_goal
            pose2 = self.amcl_pose

            dx = pose1[0] - pose2[0]
            dy = pose1[1] - pose2[1]
            distance = math.sqrt(dx**2 + dy**2)

            if distance <= tolerance:
                return True
        
        return False


    def move_base_status_callback(self, msg:GoalStatusArray):
        if msg.status_list:
            goal_status = msg.status_list[0].status
            if goal_status == 3:
                if self.active_state == 'STATE_Walking' and not self.goal_reached and self.is_stop():
                    next_goal = next(self.goal_iterator)
                    self.move_to_goal(next_goal)
                    self.goal_reached = True
                    # self.new_goal_assigned = False
                if self.active_state in ['STATE_GoHome', 'STATE_Targeting', 'STATE_GoTrash'] and not self.goal_reached and self.is_stop():
                    print("GOAL REACHED")
                    self.goal_reached = True

            elif goal_status == 1:
                self.goal_reached = False

    def move_to_goal(self, goal):
        self.goal_reached = False
        x, y, rz, rw = goal
        pub_goal = PoseStamped()
        pub_goal.header.stamp = rospy.Time.now()
        pub_goal.header.frame_id = "map"

        pub_goal.pose.position.x = x
        pub_goal.pose.position.y = y

        pub_goal.pose.orientation.z = rz
        pub_goal.pose.orientation.w = rw

        rospy.loginfo(f"Sending goal point: ({goal})")
        for _ in range(3):
            self.simple_goal_pub.publish(pub_goal)

        self.last_goal = goal
        rospy.sleep(2)
        self.goal_reached = False

    def servo_command(self, number, command):
        rospy.sleep(1)
        for _ in range(3):
            if number == 1 and command == "open":
                self.servo1_pub.publish(140)
            if number == 1 and command == "close":
                self.servo1_pub.publish(60)
            if number == 2 and command == "open":
                self.servo2_pub.publish(140)
            if number == 2 and command == "close":
                self.servo2_pub.publish(60)
        rospy.sleep(1)

    def cancel_goal(self):
        self.cancel_pub.publish(GoalID())
        self.last_goal = None
    
    def move(self, x=0, z=0):
        cmd = Twist()
        cmd.linear.x = x
        cmd.angular.z = z
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(1)

    def check_trash_and_move(self):
        while True:
            if self.detect_bounding_box:
                print("gkjghwohgw")
                print(self.detect_bounding_box)
                x1, y1, x2, y2 = self.detect_bounding_box

                if (x1 + x2) / 2 < 0.4:
                    self.move(z=0.3)
                elif (x1 + x2) / 2 > 0.6:
                    self.move(z=-0.3)
                else:
                    break

                # if y2 < 0.8:
                #     self.move(x=1)
            else:
                break


class BaseState(smach.State):
    def __init__(self, trash_turtle: TrashTurtle, name: str, outcomes: list):
        self.trash_turtle = trash_turtle
        self.state_name = name
        self.outcomes = outcomes
        smach.State.__init__(self, outcomes=self.outcomes)

    def execute(self, userdata):
        self.trash_turtle.enter_new_state()

        while not rospy.is_shutdown():
            self.trash_turtle.pub_activate_state()
            if self.trash_turtle.command in self.outcomes:
                return self.trash_turtle.command

            rospy.sleep(0.1)


class State_Sleeping(BaseState):
    def __init__(self, trash_turtle: TrashTurtle, name: str, outcomes: list):
        super(State_Sleeping, self).__init__(trash_turtle, name, outcomes)

    def execute(self, userdata):
        self.trash_turtle.enter_new_state()
        # Reset goal iter
        self.trash_turtle.goal_iterator = cycle(self.trash_turtle.goals)

        while not rospy.is_shutdown():
            self.trash_turtle.pub_activate_state()
            if self.trash_turtle.command in self.outcomes:
                return self.trash_turtle.command

            rospy.sleep(0.1)


class State_Walking(BaseState):
    def __init__(self, trash_turtle: TrashTurtle, name: str, outcomes: list):
        super(State_Walking, self).__init__(trash_turtle, name, outcomes)

    def execute(self, userdata):
        self.trash_turtle.enter_new_state()

        next_goal = next(self.trash_turtle.goal_iterator)
        self.trash_turtle.move_to_goal(next_goal)

        while not rospy.is_shutdown():
            self.trash_turtle.pub_activate_state()
            if self.trash_turtle.command in self.outcomes:
                return self.trash_turtle.command
            
            # print(self.trash_turtle.goal_reached)

            # if self.trash_turtle.goal_reached and not self.trash_turtle.new_goal_assigned:
            #     next_goal = next(self.trash_turtle.goal_iterator)
            #     self.trash_turtle.move_to_goal(next_goal)
                
            #     self.trash_turtle.goal_reached = False
            #     self.trash_turtle.new_goal_assigned = True
            #     print(self.trash_turtle.goal_reached)
            #     print("here")

            rospy.sleep(0.1)


class State_GoHome(BaseState):
    def __init__(self, trash_turtle: TrashTurtle, name: str, outcomes: list):
        super(State_GoHome, self).__init__(trash_turtle, name, outcomes)

    def execute(self, userdata):
        self.trash_turtle.enter_new_state()

        self.trash_turtle.move_to_goal(self.trash_turtle.home_position)

        while not rospy.is_shutdown():
            self.trash_turtle.pub_activate_state()
            if self.trash_turtle.command in self.outcomes:
                return self.trash_turtle.command

            if self.trash_turtle.goal_reached:
                print("I'm HOME")
                return 'sleeping'
            rospy.sleep(0.1)


class State_Targeting(BaseState):
    def __init__(self, trash_turtle: TrashTurtle, name: str, outcomes: list):
        super(State_Targeting, self).__init__(trash_turtle, name, outcomes)

    def execute(self, userdata):
        self.trash_turtle.enter_new_state()

        print("I see a TRASH")

        self.trash_turtle.cancel_goal()

        rospy.sleep(5)

        if self.trash_turtle.trash_pose:
            self.trash_turtle.check_trash_and_move()
            rospy.sleep(5)
            self.trash_turtle.move_to_goal(self.trash_turtle.trash_pose)
        else:
            return 'walking'
            # self.trash_turtle.move_to_goal(self.trash_turtle.last_goal)

        while not rospy.is_shutdown():
            self.trash_turtle.pub_activate_state()
            if self.trash_turtle.command in self.outcomes:
                return self.trash_turtle.command
            
            if self.trash_turtle.goal_reached:
                return 'picking'

            rospy.sleep(0.1)


class State_Picking(BaseState):
    def __init__(self, trash_turtle: TrashTurtle, name: str, outcomes: list):
        super(State_Picking, self).__init__(trash_turtle, name, outcomes)

    def execute(self, userdata):
        self.trash_turtle.enter_new_state()
        print("Let's get this trash.")
        self.trash_turtle.servo_command(1, "open")

        self.trash_turtle.check_trash_and_move()

        self.trash_turtle.move(x=1)
        self.trash_turtle.move(x=1)
        self.trash_turtle.move(z=-0.75)
        self.trash_turtle.move(x=0.8)
        self.trash_turtle.move(z=1)

        self.trash_turtle.servo_command(1, "close")
        return 'go_trash'


class State_GoTrash(BaseState):
    def __init__(self, trash_turtle: TrashTurtle, name: str, outcomes: list):
        super(State_GoTrash, self).__init__(trash_turtle, name, outcomes)

    def execute(self, userdata):
        self.trash_turtle.enter_new_state()

        self.trash_turtle.move_to_goal(self.trash_turtle.trash_position)

        while not rospy.is_shutdown():
            self.trash_turtle.pub_activate_state()
            if self.trash_turtle.command in self.outcomes:
                return self.trash_turtle.command

            if self.trash_turtle.goal_reached:
                print("I'm AT TRASH")
                return 'placing'
            rospy.sleep(0.1)


class State_Placing(BaseState):
    def __init__(self, trash_turtle: TrashTurtle, name: str, outcomes: list):
        super(State_Placing, self).__init__(trash_turtle, name, outcomes)

    def execute(self, userdata):
        self.trash_turtle.enter_new_state()

        self.trash_turtle.servo_command(1, "open")
        self.trash_turtle.servo_command(2, "open")
        self.trash_turtle.servo_command(2, "close")
        self.trash_turtle.servo_command(1, "close")
        return 'walking'


# Main function
def main():
    rospy.init_node('state_machine')

    trash_turtle = TrashTurtle()

    # Create a SMACH state machine
    with trash_turtle.sm:
        trash_turtle.add_state(
            'STATE_Sleeping', State_Sleeping, outcomes=['walking'])
        trash_turtle.add_state('STATE_Walking', State_Walking, outcomes=[
                               'go_home', 'targeting'])
        trash_turtle.add_state('STATE_GoHome', State_GoHome, outcomes=[
                               'walking', 'sleeping'])
        trash_turtle.add_state('STATE_Targeting', State_Targeting, outcomes=[
                               'go_home', 'walking', 'picking'])
        trash_turtle.add_state('STATE_Picking', State_Picking, outcomes=[
                               'walking', 'go_trash'])
        trash_turtle.add_state(
            'STATE_GoTrash', State_GoTrash, outcomes=['placing'])
        trash_turtle.add_state('STATE_Placing', State_Placing, outcomes=[
                               'walking', 'go_home'])

    # Create and start the introspection server (for visualizing the state machine)
    sis = smach_ros.IntrospectionServer(
        'smach_server', trash_turtle.sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = trash_turtle.sm.execute()

    # Wait for Ctrl+C to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    print("LET'S DANCE..")
    main()
