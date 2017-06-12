"""
author: Philippe Lebel, based on Vincent Babin's work,

Robotic's laboratory, mechanical engineering dept., Laval University, Canada

date: 09/06/17

Computes inverse kinematics of the Jaco1 robot.
inputs:
    t_target:       cartesian coordinates of the end-effector (in meters). np.array([[x],[y],[z]])
    Q_target:       rotation matrix of the end effector. np.array([[1,0,0],[0,1,0],[0,0,1]])
    ThetaEstimate:  joint angles NEAR the desired cartesian position and orientation. (radians) np.array.shape = (6,1)
Outputs:
    Output:         Joint angles to reach wanted position and orientation. np.array.shape = (6,1)
                        (it outputs ThetaEstimate if max_iter is reached)
    sol:            Bool, True if converged, False if not.
    SolApprox:      Last solution computed if not converged. Same as Output if converged. np.array.shape = (6,1)


As this method is not an analytic one, it is important that ThetaEstimate is near the desired positions as it will
diverge otherwise.
Function has a maximum of 30 iterations to converge, it will output it's last result if max_iters is reached.
"""


import numpy as np


def inverse_kinematics_jaco(t_target, Q_target, ThetaEstimate):

    # Local Variables: stop, Q1, EQ1, d4b, EQ3, EQ2, EQ5, EQ4, EQ6, R1, R2, R3, dtheta, d, Jinv,
    #                  SolApprox, d6b, Hk, E2, P2, P3, P1, P6, P4, P5, e5, e4, e6, e1, e3, e2, r4, r5, r6, r1, Ek, r3,
    #                  sample_time, d5b, erreur, Wn, theta2, theta3, theta1, theta6, theta4, theta5, sol, Output, Q6, Q,
    #                  epsilon, J, ThetaEstimate, r2, f, aa, Q3, Q2, Q5, Q4, Q_target, JointOutput, s2a, s3, s2, s1,
    #                  t_target, E, a1, iter_max, a3, a2, a5, a4, a6, alpha, sa, a, Weight, D6, D4, D5, D2, D3, D1
    # Function calls: cos, inverse_kinematics_jaco, eye, max, cross, abs, zeros, pi, sin

    JointOutput = ThetaEstimate.reshape(6, 1)

    # Parameters of Jaco robot
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
    sample_time = 0.005
    #%Testing Purposes ---------------------------------------
    iter_max = 30
    f = np.zeros(12).reshape(12, 1)
    Output = np.array([[0], [0], [0], [0], [0], [0]])
    sol = False
    SolApprox = np.array([[0], [0], [0], [0], [0], [0]])
    for iter in range(iter_max):
        theta1 = JointOutput[0, 0]
        theta2 = JointOutput[1, 0]
        theta3 = JointOutput[2, 0]
        theta4 = JointOutput[3, 0]
        theta5 = JointOutput[4, 0]
        theta6 = JointOutput[5, 0]
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
        P6 = np.dot(P5, Q6)
        Q = P6
        e1 = np.array(np.hstack((0., 0., 1.))).conj().T
        e2 = np.dot(P1, e1)
        e3 = np.dot(P2, e1)
        e4 = np.dot(P3, e1)
        e5 = np.dot(P4, e1)
        e6 = np.dot(P5, e1)
        r1 = a1+np.dot(P1, a2)+np.dot(P2, a3)+np.dot(P3, a4)+np.dot(P4, a5)+np.dot(P5, a6)
        r2 = np.dot(P1, a2)+np.dot(P2, a3)+np.dot(P3, a4)+np.dot(P4, a5)+np.dot(P5, a6)
        r3 = np.dot(P2, a3)+np.dot(P3, a4)+np.dot(P4, a5)+np.dot(P5, a6)
        r4 = np.dot(P3, a4)+np.dot(P4, a5)+np.dot(P5, a6)
        r5 = np.dot(P4, a5)+np.dot(P5, a6)
        r6 = np.dot(P5, a6)
        E = np.array(np.vstack((np.hstack((0., -1., 0.)), np.hstack((1., 0., 0.)), np.hstack((0., 0., 0.)))))
        s1 = np.array(np.hstack((1., 0., 0.))).conj().T
        s2 = np.array(np.hstack((0., 1., 0.))).conj().T
        s3 = np.array(np.hstack((0., 0., 1.))).conj().T
        EQ1 = np.dot(E, Q)
        EQ2 = np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(P1, E), Q2), Q3), Q4), Q5), Q6)
        EQ3 = np.dot(np.dot(np.dot(np.dot(np.dot(P2, E), Q3), Q4), Q5), Q6)
        EQ4 = np.dot(np.dot(np.dot(np.dot(P3, E), Q4), Q5), Q6)
        EQ5 = np.dot(np.dot(np.dot(P4, E), Q5), Q6)
        EQ6 = np.dot(np.dot(P5, E), Q6)

        R1 = np.array(np.hstack((np.dot(EQ1, s1).reshape(3, 1),
                                 np.dot(EQ2, s1).reshape(3, 1),
                                 np.dot(EQ3, s1).reshape(3, 1),
                                 np.dot(EQ4, s1).reshape(3, 1),
                                 np.dot(EQ5, s1).reshape(3, 1),
                                 np.dot(EQ6, s1).reshape(3, 1))))
        R2 = np.array(np.hstack((np.dot(EQ1, s2).reshape(3, 1),
                                 np.dot(EQ2, s2).reshape(3, 1),
                                 np.dot(EQ3, s2).reshape(3, 1),
                                 np.dot(EQ4, s2).reshape(3, 1),
                                 np.dot(EQ5, s2).reshape(3, 1),
                                 np.dot(EQ6, s2).reshape(3, 1))))
        R3 = np.array(np.hstack((np.dot(EQ1, s3).reshape(3, 1),
                                 np.dot(EQ2, s3).reshape(3, 1),
                                 np.dot(EQ3, s3).reshape(3, 1),
                                 np.dot(EQ4, s3).reshape(3, 1),
                                 np.dot(EQ5, s3).reshape(3, 1),
                                 np.dot(EQ6, s3).reshape(3, 1))))
        # Jacobian matrix
        J = np.vstack((np.vstack((R1, R2, R3)),
                       np.hstack((np.cross(e1, r1.transpose()).reshape(3, 1),
                                  np.cross(e2, r2.transpose()).reshape(3, 1),
                                  np.cross(e3, r3.transpose()).reshape(3, 1),
                                  np.cross(e4, r4.transpose()).reshape(3, 1),
                                  np.cross(e5, r5.transpose()).reshape(3, 1),
                                  np.cross(e6, r6.transpose()).reshape(3, 1)))))

        # -- Salution's fitness --------------------------------------------------------------------------------------
        f[0:3, 0] = np.dot(Q-Q_target, s1)
        f[3:6, 0] = np.dot(Q-Q_target, s2)
        f[6:9, 0] = np.dot(Q-Q_target, s3)
        f[9:12, 0] = (r1-t_target).reshape(3)
        # -- Stop criteria -------------------------------------------------------------------------------------------
        epsilon = 0.01
        stop = np.max(np.abs(f)) < epsilon

        Ek = np.dot(np.multiply(0.5, f.conj().T)[0], f)
        Wn = np.dot(Ek[0], np.eye(6)) + np.multiply(0.001, np.eye(6))
        Hk = np.dot(J.conj().T, J) + Wn
        Jinv = np.dot(np.linalg.inv(Hk), J.conj().T)
        dtheta = np.dot(Jinv, f)
        JointOutput = JointOutput - dtheta
        SolApprox = JointOutput
        if stop:
            sol = True
            Output = JointOutput
            return Output, sol, Output
        else:
            sol = False
            Output = ThetaEstimate
    Output = Output.reshape(6, 1)
    SolApprox = SolApprox.reshape(6, 1)
    return Output, sol, SolApprox