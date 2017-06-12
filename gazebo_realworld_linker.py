import argparse
from joint_state_velocity_publisher import *
import pygame
import roslib; roslib.load_manifest('kinova_demo')
import rospy
import sys
import math
import actionlib
import kinova_msgs.msg
from sensor_msgs.msg import JointState
from joint_states_listener_real_jaco.srv import ReturnJointStatesRealJaco



def argumentParser(argument):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Either use joystick to move robot in gazebo or links a real robot to '
                                                 'the gazebo robot')
    parser.add_argument('Control_type', metavar='control_type', type=str, default='gazebo',
                        help='Control_type = Gazebo: Only in gazebo environement. Control_type = real_world: links real'
                             'robot to gazebo robot')
    args_ = parser.parse_args(argument)
    return args_


#thread function: listen for joint_states messages
def joint_states_listener(self):
    rospy.Subscriber('/jaco/joint_state', JointState, self.joint_states_callback)
    rospy.spin()


#callback function: when a joint_states message arrives, save the values
def joint_states_callback(self, msg):
    self.lock.acquire()
    self.name = msg.name
    self.position = msg.position
    self.velocity = msg.velocity
    self.effort = msg.effort
    self.lock.release()


if __name__ == '__main__':
    args = argumentParser(None)
    if args.control_type == 'gazebo':
        try:
            pygame.init()
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            s = JointStatePublisher(joystick)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    elif args.control_type == 'real_world':
        pass
    else:
        print('wrong inputs try "-h" to get help')


    """
    if __name__ == '__main__':

        args = argumentParser(None)

        kinova_robotTypeParser(args.kinova_robotType)
        rospy.init_node(prefix + 'gripper_workout')

        # currentJointCommand = [0]*arm_joint_number
        # KinovaType defines AngularInfo has 7DOF, so for published topics on joints.
        currentJointCommand = [0] * 7
        print(args)
        if len(args.joint_value) != arm_joint_number:
            print(
            'Number of input values {} is not equal to number of joints {}. Please run help to check number of joints with different robot type.'.format(
                len(args.joint_value), arm_joint_number))
            sys.exit(0)

        # get Current finger position if relative position
        getcurrentJointCommand(prefix)
        joint_degree, joint_radian = unitParser(args.unit, args.joint_value, args.relative)

        positions = [0] * 7
        try:

            if arm_joint_number < 1:
                print('Joint number is 0, check with "-h" to see how to use this node.')
                positions = []  # Get rid of static analysis warning that doesn't see the exit()
                sys.exit()
            else:
                for i in range(0, arm_joint_number):
                    positions[i] = joint_degree[i]

            result = joint_angle_client(positions)

        except rospy.ROSInterruptException:
            print('program interrupted before completion')

        verboseParser(args.verbose, joint_degree)
    """