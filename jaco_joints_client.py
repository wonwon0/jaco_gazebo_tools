

import rospy
import kinova_msgs.msg
import time
import threading
import numpy as np


class JacoJointsClient:
    def __init__(self):
        self.lock = threading.Lock()
        self.theta = []
        self.fingers_positions = []
        self.thread_joints = threading.Thread(target=self.joint_states_listener)
        self.thread_fingers = threading.Thread(target=self.finger_states_listener)
        self.cond = threading.Condition(self.lock)
        self.thread_joints.start()
        self.thread_fingers.start()
        self.timeout = 0.01

    def joint_states_listener(self):
        rospy.Subscriber('/j2n6s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, self.gimme_joints)
        rospy.spin()

    def finger_states_listener(self):
        rospy.Subscriber('/j2n6s300_driver/out/finger_position', kinova_msgs.msg.FingerPosition, self.gimme_fingers)

    def gimme_joints(self, msg):
        self.lock.acquire()
        current_time = start_time = time.time()
        if msg == []:
            self.cond.wait(self.timeout - current_time + start_time)
        self.theta = [msg.joint1 * np.pi / 180., msg.joint2 * np.pi / 180., msg.joint3 * np.pi / 180.,
                      msg.joint4 * np.pi / 180., msg.joint5 * np.pi / 180., msg.joint6 * np.pi / 180.]
        self.lock.release()

    def gimme_fingers(self, msg):
        self.lock.acquire()
        current_time = start_time = time.time()
        if msg == []:
            self.cond.wait(self.timeout - current_time + start_time)
        self.fingers_positions = [msg.finger1 * np.pi / 180., msg.finger2 * np.pi / 180., msg.finger3 * np.pi / 180.]
        self.lock.release()

    def get_joints_states(self):
        if self.theta == []:
            rospy.logerr("no jaco packets for angles received")
            return self.theta
        self.lock.acquire()
        joints_angles = self.theta
        self.lock.release()
        return joints_angles

    def get_fingers_states(self):
        if self.fingers_positions == []:
            rospy.logerr("no jaco packets for fingers received")
            return self.fingers_positions
        self.lock.acquire()
        fingers_angles = self.fingers_positions
        self.lock.release()
        return fingers_angles


def talker():
    rospy.init_node('pls_work', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    jaco = JacoJointsClient()
    start = time.time()
    while not rospy.is_shutdown():
        rate.sleep()
        print(jaco.get_joints_states())
        dt = time.time() - start
        print(dt)
        start = time.time()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



