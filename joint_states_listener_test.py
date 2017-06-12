#!/usr/bin/env python
#test client for joint_states_listener

import roslib
roslib.load_manifest('joint_states_listener_real_jaco')
import rospy
from joint_states_listener_real_jaco.srv import ReturnJointStatesRealJaco
import time
import sys

def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states_real_jaco", ReturnJointStatesRealJaco)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    return theta


#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


#print out the positions, velocities, and efforts of the right arm joints
if __name__ == "__main__":
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    while(1):
        theta = call_return_joint_states(joint_names)
        print "position:", theta
        time.sleep(1)