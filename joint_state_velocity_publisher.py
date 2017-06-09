#!/usr/bin/env python
# license removed for brevity


import pygame
import roslib
import math
import numpy as np
import sys
from PGDVince import PGDVince
from PGIVince import PGIVince
roslib.load_manifest('joint_states_listener')
from joint_states_listener.srv import ReturnJointStates
import rospy
from sensor_msgs.msg import JointState
import jaco_jacobian

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher():
    def __init__(self, joystick=None):
        self.joystick = joystick
        rospy.init_node('dynamixel_joint_state_publisher', anonymous=True)

        self.rate = 100
        rate = rospy.get_param('~rate', self.rate)
        r = rospy.Rate(rate)


        # The namespace and joints parameter needs to be set by the servo controller
        # (The namespace is usually null.)
        namespace = "jaco_on_table::"
        self.joints = ["jaco_arm_0_joint",
                        "jaco_arm_1_joint",
                        "jaco_arm_2_joint",
                        "jaco_arm_3_joint",
                        "jaco_arm_4_joint",
                        "jaco_arm_5_joint"]
        self.latest_states = JointStatesListener(self.joints)
        self.joints = [namespace + "jaco_arm_0_joint",
                       namespace + "jaco_arm_1_joint",
                       namespace + "jaco_arm_2_joint",
                       namespace + "jaco_arm_3_joint",
                       namespace + "jaco_arm_4_joint",
                       namespace + "jaco_arm_5_joint"]

        self.servos = list()
        self.controllers = list()
        self.last_working_pose = None
        # self.joint_states = dict({})

        for controller in self.joints:
            # self.joint_states[controller] = JointStateMessage(controller, 0.0, 0.0, 0.0)
            self.controllers.append(controller)

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/jaco/joint_control', JointState)

        rospy.loginfo("Starting Joint State Publisher at " + str(rate) + "Hz")

        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    # def controller_state_handler(self, msg):
    #     js = JointStateMessage(msg.name, msg.current_pos, msg.velocity, msg.load)
    #     self.joint_states[msg.name] = js

    def publish_joint_states(self):
        pygame.event.pump()
        # Construct message & publish joint states

        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        if self.joystick is None:
            velocities = [0, 0, 0, 0, 0, 0]
        else:
            velocities = [self.joystick.get_axis(0) / 10.0,
                          self.joystick.get_axis(1) / 10.0,
                          self.joystick.get_axis(2) / 10.0, 0, 0, 0]
        [latest_positions, latest_velocities, latest_effort] = self.latest_states.call_return_joint_states()
        if self.last_working_pose is None:
            self.last_working_pose = latest_positions
        angular_velocities = convert_to_angular_velocities(velocities, latest_positions, 1.0 / self.rate)
        angular_velocities = angular_velocities.reshape(1, 6)[0]
        #print(angular_velocities)
        for idx, controller in enumerate(self.controllers):
            msg.name.append(controller)
            if abs(angular_velocities[idx]) <= 0.00000001:
                msg.position.append(self.last_working_pose[idx])
                #msg.velocity.append(0)
            else:
                self.last_working_pose = latest_positions
                msg.position.append(latest_positions[idx] + velocities[idx] * (1.0 / self.rate))
                msg.velocity.append(angular_velocities[idx])
        self.joint_states_pub.publish(msg)

            #msg.position.append(joint.position)
            #msg.velocity.append(velocities[idx])
            #msg.effort.append(joint.effort)



class JointStatesListener:
    def __init__(self, joint_names):
        self.joint_names = joint_names

    def call_return_joint_states(self):
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(self.joint_names)
        except rospy.ServiceException, e:
            print "error when calling return_joint_states: %s" % e
            sys.exit(1)
        for (ind, joint_name) in enumerate(self.joint_names):
            if (not resp.found[ind]):
                print "joint %s not found!" % joint_name
        return (resp.position, resp.velocity, resp.effort)


def convert_to_angular_velocities(velocities, theta_current, dt):
    velocities = np.array([[velocities[0]],
                            [velocities[1]],
                            [velocities[2]],
                            [velocities[3]],
                            [velocities[4]],
                            [velocities[5]]])
    theta_current = np.array([[theta_current[0]],
                               [theta_current[1]],
                               [theta_current[2]],
                               [theta_current[3]],
                               [theta_current[4]],
                               [theta_current[5]]])
    cartesian_pose, rotation_matrix = PGDVince(theta_current)
    next_cartesian_pose = cartesian_pose + np.multiply(velocities[0:3, 0].reshape(3, 1), dt)
    euler_angles = rotationMatrixToEulerAngles(rotation_matrix)
    next_euler_angles = np.array(euler_angles).reshape(3, 1) + np.multiply(velocities[3:6, 0].reshape(3, 1), dt)
    next_rotation_matrix = eulerAnglesToRotationMatrix([next_euler_angles[0, 0],
                                                        next_euler_angles[1, 0],
                                                        next_euler_angles[2, 0]])

    next_theta, success, sol_approx = PGIVince(next_cartesian_pose, rotation_matrix, theta_current)
    # print(np.unwrap(theta_current), "theta_current")
    # print(np.unwrap(next_theta), "next_theta")
    # print(np.subtract(np.unwrap(next_theta), np.unwrap(theta_current)), "theta_diff")
    # print(dt)
    # next_theta[1, 0] *= -1.0
    angular_velocity = np.divide(np.subtract(next_theta, theta_current), dt)
    for idx, velocity in enumerate(angular_velocity.reshape(1, 6)[0]):
        if idx == 1 or idx == 3 or idx == 4 or idx == 5:
            angular_velocity[idx] = -velocity
        if velocity*dt > np.pi:
            angular_velocity[idx] = velocity - np.pi/dt
        elif velocity*dt < -np.pi:
            angular_velocity[idx] = velocity + np.pi / dt
    return angular_velocity


# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


if __name__ == '__main__':
    try:
        pygame.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        s = JointStatePublisher(joystick)
        rospy.spin()
    except rospy.ROSInterruptException: pass




