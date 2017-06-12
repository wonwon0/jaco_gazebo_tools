#!/usr/bin/env python
#spins off a thread to listen for joint_states messages
#and provides the same information (or subsets of) as a service

import roslib
roslib.load_manifest('joint_states_listener')
import rospy
from joint_states_listener_real_jaco.srv import *
from sensor_msgs.msg import JointState
import kinova_msgs.msg
import threading


#holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self):
        rospy.init_node('joint_states_listener_real_jaco')
        self.lock = threading.Lock()
        self.theta = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()
        s = rospy.Service('return_joint_states', ReturnJointStatesRealJaco, self.return_joint_states)


    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('/j2n6s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, self.joint_states_callback)
        rospy.spin()


    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        self.theta = [msg.joint1, msg.joint1, msg.joint1, msg.joint1, msg.joint1, msg.joint1]
        self.lock.release()


    #returns (found, position, velocity, effort) for the joint joint_name
    #(found is 1 if found, 0 otherwise)
    def return_joint_state(self, theta_id):

        #no messages yet
        if self.theta == []:
            rospy.logerr("No robot_state messages received!\n")
            return []

        #return info for this joint
        self.lock.acquire()
        if len(self.theta) <= (theta_id + 1):
            theta = self.theta[theta_id]
        #unless it's not found
        else:
            rospy.logerr("Joint %s not found!", (theta_id,))
            self.lock.release()
            return (0, 0., 0., 0.)
        self.lock.release()
        return theta


    #server callback: returns arrays of position, velocity, and effort
    #for a list of joints specified by name
    def return_joint_states(self, req):
        thetas = []
        for joint in req:
            theta = self.return_joint_state(joint)
            thetas.append(theta)
        return ReturnJointStatesResponseRealJaco(thetas)


#run the server
if __name__ == "__main__":

    latestjointstates = LatestJointStates()

    print "joints_states_listener server started, waiting for queries"
    rospy.spin()