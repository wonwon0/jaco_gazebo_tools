

import argparse
from joint_states_publisher import *
import pygame
import rospy



def argumentParser(argument):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Either use joystick to move robot in gazebo or links a real robot to '
                                                 'the gazebo robot')
    parser.add_argument('control_type', metavar='control_type', type=str, default='gazebo',
                        help='control_type = Gazebo: Only in gazebo environement. Control_type = real_world: links real'
                             'robot to gazebo robot')
    args_ = parser.parse_args(argument)
    return args_

if __name__ == '__main__':
    args = argumentParser(None)
    if (args.control_type == 'gazebo') or (args.control_type == 'real_world'):
        try:
            pygame.init()
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            s = JointStatePublisher(joystick, args.control_type)
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    else:
        print('wrong inputs try "-h" to get help')