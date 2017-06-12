
import argparse

from kinova_demo.robot_control_modules import joint_position_client

from joint_state_velocity_publisher import *
import pygame
import rospy
from sensor_msgs.msg import JointState
import roslib; roslib.load_manifest('kinova_demo')
import rospy
import sys
import math
import actionlib
import kinova_msgs.msg
import time


def dummy(msg):
    print(msg)
if __name__ == '__main__':
    rospy.init_node('pls_work', anonymous=True)

    for i in range(100):
        b = time.time()
        a = rospy.Subscriber('/j2n6s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, dummy)
        time.sleep(0.1)
        print(time.time() - b)

