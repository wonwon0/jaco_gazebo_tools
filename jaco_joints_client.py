

import rospy
import kinova_msgs.msg
import time
import threading
import numpy as np


class JacoJointsClient:
    def __init__(self):
        self.lock = threading.Lock()
        self.theta = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.cond = threading.Condition(self.lock)
        self.thread.start()
        self.timeout = 0.01

    def joint_states_listener(self):
        rospy.Subscriber('/j2n6s300_driver/out/joint_angles', kinova_msgs.msg.JointAngles, self.gimme_joints)
        rospy.spin()

    def gimme_joints(self, msg):
        self.lock.acquire()
        current_time = start_time = time.time()
        if msg is []:
            self.cond.wait(self.timeout - current_time + start_time)
        self.theta = [msg.joint1 * np.pi / 180., msg.joint2 * np.pi / 180., msg.joint3 * np.pi / 180.,
                      msg.joint3 * np.pi / 180., msg.joint5 * np.pi / 180., msg.joint6 * np.pi / 180.]
        self.lock.release()

    def get_joints_states(self):
        if self.theta is []:
            rospy.logerr("no jaco packets received")
            return self.theta

        self.lock.acquire()
        joints_angles = self.theta
        self.lock.release()
        return joints_angles


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



