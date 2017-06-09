"""
author: Philippe Lebel, based on Vincent Babin's work,

Robotic's laboratory, mechanical engineering dept., Laval University, Canada

date: 09/06/17

Computes direct kinematics of the Jaco1 robot.
inputs:
    ThetaEncoders:  joint angles of the robot (radians) np.array.shape = (6,1)
Outputs:
    t_real:       cartesian coordinates of the end-effector (in meters). np.array([[x],[y],[z]])
    Q_real:       rotation matrix of the end effector. np.array([[1,0,0],[0,1,0],[0,0,1]])

"""

import numpy as np


def direct_kinematics_jaco(ThetaEncoders):

    # Local Variables: aa, theta3, Q3, theta1, theta6, Q4, theta4, theta5, Q2, Q5, Q1, s2a, d4b, Q6, d6b, ThetaEncoders,
    #                  Q_real, t_real, E2, P2, P3, P1, P4, P5, a1, a3, a2, a5, a4, a6, alpha, D4, a, d, d5b, theta2,
    #                  D6, sa, D5, D2, D3, D1
    # Function calls: direct_kinematics_jaco, cos, pi, sin

    # Parameters of Jaco robot
    ThetaEncoders = ThetaEncoders.reshape(6, 1)
    ThetaEncoders[1, 0] *= -1.0
    D1 = 0.2755
    D2 = 0.41
    D3 = 0.2073
    D4 = 0.0743
    D5 = 0.0743
    D6 = 0.1687
    E2 = 0.0098
    aa = 11.*np.pi/72.
    # intermediate variables
    sa = np.sin(aa)
    s2a = np.sin((2.*aa))
    d4b = D3 + (sa / s2a) * D4
    d5b = (sa / s2a) * D4 + (sa / s2a) * D5
    d6b = (sa / s2a) * D5 + D6
    # DH parameters(Kinova)
    a = np.array(np.hstack((0., D2, 0., 0., 0., 0.)))
    d = np.array(np.hstack((D1, 0., -E2, -d4b, -d5b, -d6b)))
    alpha = np.array(np.hstack((np.pi/2., np.pi, np.pi/2., 2.*aa, 2.*aa, np.pi)))

    theta1 = ThetaEncoders[0, 0]
    theta2 = ThetaEncoders[1, 0]
    theta3 = ThetaEncoders[2, 0]
    theta4 = ThetaEncoders[3, 0]
    theta5 = ThetaEncoders[4, 0]
    theta6 = ThetaEncoders[5, 0]
    a1 = np.array(np.vstack((np.hstack([(np.dot(a[0], np.cos(theta1)))]),
                             np.hstack([(np.dot(a[0], np.sin(theta1)))]),
                             np.hstack([d[0]]))))
    a2 = np.array(np.vstack((np.hstack([(np.dot(a[1], np.cos(theta2)))]),
                             np.hstack([(np.dot(a[1], np.sin(theta2)))]),
                             np.hstack([d[1]]))))
    a3 = np.array(np.vstack((np.hstack([(np.dot(a[2], np.cos(theta3)))]),
                             np.hstack([(np.dot(a[2], np.sin(theta3)))]),
                             np.hstack([d[2]]))))
    a4 = np.array(np.vstack((np.hstack([(np.dot(a[3], np.cos(theta4)))]),
                             np.hstack([(np.dot(a[3], np.sin(theta4)))]),
                             np.hstack([d[3]]))))
    a5 = np.array(np.vstack((np.hstack([(np.dot(a[4], np.cos(theta5)))]),
                             np.hstack([(np.dot(a[4], np.sin(theta5)))]),
                             np.hstack([d[4]]))))
    a6 = np.array(np.vstack((np.hstack([(np.dot(a[5], np.cos(theta6)))]),
                             np.hstack([(np.dot(a[5], np.sin(theta6)))]),
                             np.hstack([d[5]]))))
    Q1 = np.array(np.vstack((np.hstack((np.cos(theta1), np.dot(-np.cos(alpha[0]), np.sin(theta1)),
                                        np.dot(np.sin(alpha[0]), np.sin(theta1)))),
                             np.hstack((np.sin(theta1), np.dot(np.cos(alpha[0]), np.cos(theta1)),
                                        np.dot(-np.sin(alpha[0]), np.cos(theta1)))),
                             np.hstack((0., np.sin(alpha[0]), np.cos(alpha[0]))))))

    Q2 = np.array(np.vstack((np.hstack((np.cos(theta2), np.dot(-np.cos(alpha[1]), np.sin(theta2)),
                                        np.dot(np.sin(alpha[1]), np.sin(theta2)))),
                             np.hstack((np.sin(theta2), np.dot(np.cos(alpha[1]), np.cos(theta2)),
                                        np.dot(-np.sin(alpha[1]), np.cos(theta2)))),
                             np.hstack((0., np.sin(alpha[1]), np.cos(alpha[1]))))))

    Q3 = np.array(np.vstack((np.hstack((np.cos(theta3), np.dot(-np.cos(alpha[2]), np.sin(theta3)),
                                        np.dot(np.sin(alpha[2]), np.sin(theta3)))),
                             np.hstack((np.sin(theta3), np.dot(np.cos(alpha[2]), np.cos(theta3)),
                                        np.dot(-np.sin(alpha[2]), np.cos(theta3)))),
                             np.hstack((0., np.sin(alpha[2]), np.cos(alpha[2]))))))

    Q4 = np.array(np.vstack((np.hstack((np.cos(theta4), np.dot(-np.cos(alpha[3]), np.sin(theta4)),
                                        np.dot(np.sin(alpha[3]), np.sin(theta4)))),
                             np.hstack((np.sin(theta4), np.dot(np.cos(alpha[3]), np.cos(theta4)),
                                        np.dot(-np.sin(alpha[3]), np.cos(theta4)))),
                             np.hstack((0., np.sin(alpha[3]), np.cos(alpha[3]))))))

    Q5 = np.array(np.vstack((np.hstack((np.cos(theta5), np.dot(-np.cos(alpha[4]), np.sin(theta5)),
                                        np.dot(np.sin(alpha[4]), np.sin(theta5)))),
                             np.hstack((np.sin(theta5), np.dot(np.cos(alpha[4]), np.cos(theta5)),
                                        np.dot(-np.sin(alpha[4]), np.cos(theta5)))),
                             np.hstack((0., np.sin(alpha[4]), np.cos(alpha[4]))))))

    Q6 = np.array(np.vstack((np.hstack((np.cos(theta6), np.dot(-np.cos(alpha[5]), np.sin(theta6)),
                                        np.dot(np.sin(alpha[5]), np.sin(theta6)))),
                             np.hstack((np.sin(theta6), np.dot(np.cos(alpha[5]), np.cos(theta6)),
                                        np.dot(-np.sin(alpha[5]), np.cos(theta6)))),
                             np.hstack((0., np.sin(alpha[5]), np.cos(alpha[5]))))))
    P1 = Q1
    P2 = np.dot(P1, Q2)
    P3 = np.dot(P2, Q3)
    P4 = np.dot(P3, Q4)
    P5 = np.dot(P4, Q5)
    Q_real = np.dot(P5, Q6)
    t_real = a1+np.dot(P1, a2)+np.dot(P2, a3)+np.dot(P3, a4)+np.dot(P4, a5)+np.dot(P5, a6)
    return t_real, Q_real