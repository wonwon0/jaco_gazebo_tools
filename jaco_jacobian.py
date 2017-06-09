import numpy as np
import math


def convert_to_angular_velocities(velocities, theta_current, dt):
    velocities = np.matrix([[velocities[0]],
                            [velocities[1]],
                            [velocities[2]],
                            [velocities[3]],
                            [velocities[4]],
                            [velocities[5]]])
    theta_current = np.matrix([[theta_current[0]],
                               [theta_current[1]],
                               [theta_current[2]],
                               [theta_current[3]],
                               [theta_current[4]],
                               [theta_current[5]]])
    cartesian_pose, rotation_matrix = jaco_direct_kinematics(theta_current)
    next_cartesian_pose = cartesian_pose + velocities[0:3, 0] * dt
    euler_angles = rotationMatrixToEulerAngles(rotation_matrix)
    next_euler_angles = np.matrix(euler_angles).transpose() + velocities[3:6, 0] * dt
    next_rotation_matrix = eulerAnglesToRotationMatrix([next_euler_angles[0, 0],
                                                        next_euler_angles[1, 0],
                                                        next_euler_angles[2, 0]])

    next_theta, success = jaco_inverse_kinematics(theta_current, next_cartesian_pose, next_rotation_matrix)
    # print(next_cartesian_pose, "next pose")
    # print(np.unwrap(theta_current), "theta_current")
    # print(np.unwrap(next_theta), "next_theta")
    # print(np.subtract(np.unwrap(next_theta), np.unwrap(theta_current)), "theta_diff")
    # print(dt)
    angular_velocity = np.subtract(np.unwrap(next_theta), np.unwrap(theta_current)) / dt

    return angular_velocity


def get_jaco_jacobian(theta, a, d, alpha):

    theta1 = theta[0, 0]
    theta2 = theta[1, 0]
    theta3 = theta[2, 0]
    theta4 = theta[3, 0]
    theta5 = theta[4, 0]
    theta6 = theta[5, 0]

    a1 = np.matrix([[a[0, 0] * np.cos(theta1)],
                    [a[0, 0] * np.sin(theta1)],
                    [d[0, 0]]])
    a2 = np.matrix([[a[0, 1] * np.cos(theta2)],
                    [a[0, 1] * np.sin(theta2)],
                    [d[0, 1]]])
    a3 = np.matrix([[a[0, 2] * np.cos(theta3)],
                    [a[0, 2] * np.sin(theta3)],
                    [d[0, 2]]])
    a4 = np.matrix([[a[0, 3] * np.cos(theta4)],
                    [a[0, 3] * np.sin(theta4)],
                    [d[0, 3]]])
    a5 = np.matrix([[a[0, 4] * np.cos(theta5)],
                    [a[0, 4] * np.sin(theta5)],
                    [d[0, 4]]])
    a6 = np.matrix([[a[0, 5] * np.cos(theta6)],
                    [a[0, 5] * np.sin(theta6)],
                    [d[0, 5]]])

    Q1 = np.matrix([[np.cos(theta1), -np.cos(alpha[0, 0]) * np.sin(theta1), np.sin(alpha[0, 0]) * np.sin(theta1)],
                    [np.sin(theta1), np.cos(alpha[0, 0]) * np.cos(theta1), -np.sin(alpha[0, 0]) * np.cos(theta1)],
                    [0, np.sin(alpha[0, 0]), np.cos(alpha[0, 0])]])
    Q2 = np.matrix([[np.cos(theta2), -np.cos(alpha[0, 1]) * np.sin(theta2), np.sin(alpha[0, 1]) * np.sin(theta2)],
                    [np.sin(theta2), np.cos(alpha[0, 1]) * np.cos(theta2), -np.sin(alpha[0, 1]) * np.cos(theta2)],
                    [0, np.sin(alpha[0, 1]), np.cos(alpha[0, 1])]])
    Q3 = np.matrix([[np.cos(theta3), -np.cos(alpha[0, 2]) * np.sin(theta3), np.sin(alpha[0, 2]) * np.sin(theta3)],
                    [np.sin(theta3), np.cos(alpha[0, 2]) * np.cos(theta3), -np.sin(alpha[0, 2]) * np.cos(theta3)],
                    [0, np.sin(alpha[0, 2]), np.cos(alpha[0, 2])]])
    Q4 = np.matrix([[np.cos(theta4), -np.cos(alpha[0, 3]) * np.sin(theta4), np.sin(alpha[0, 3]) * np.sin(theta4)],
                    [np.sin(theta4), np.cos(alpha[0, 3]) * np.cos(theta4), -np.sin(alpha[0, 3]) * np.cos(theta4)],
                    [0, np.sin(alpha[0, 3]), np.cos(alpha[0, 3])]])
    Q5 = np.matrix([[np.cos(theta5), -np.cos(alpha[0, 4]) * np.sin(theta5), np.sin(alpha[0, 4]) * np.sin(theta5)],
                    [np.sin(theta5), np.cos(alpha[0, 4]) * np.cos(theta5), -np.sin(alpha[0, 4]) * np.cos(theta5)],
                    [0, np.sin(alpha[0, 4]), np.cos(alpha[0, 4])]])
    Q6 = np.matrix([[np.cos(theta6), -np.cos(alpha[0, 5]) * np.sin(theta6), np.sin(alpha[0, 5]) * np.sin(theta6)],
                    [np.sin(theta6), np.cos(alpha[0, 5]) * np.cos(theta6), -np.sin(alpha[0, 5]) * np.cos(theta6)],
                    [0, np.sin(alpha[0, 5]), np.cos(alpha[0, 5])]])
    Q = Q1 * Q2 * Q3 * Q4 * Q5 * Q6
    e1 = np.matrix([[0], [0], [1]])
    e2 = Q1 * e1
    e3 = Q1 * Q2 * e1
    e4 = Q1 * Q2 * Q3 * e1
    e5 = Q1 * Q2 * Q3 * Q4 * e1
    e6 = Q1 * Q2 * Q3 * Q4 * Q5 * e1
    r1 = a1 + Q1 * a2 + Q1 * Q2 * a3 + Q1 * Q2 * Q3 * a4 + Q1 * Q2 * Q3 * Q4 * a5 + Q1 * Q2 * Q3 * Q4 * Q5 * a6
    r2 = Q1 * a2 + Q1 * Q2 * a3 + Q1 * Q2 * Q3 * a4 + Q1 * Q2 * Q3 * Q4 * a5 + Q1 * Q2 * Q3 * Q4 * Q5 * a6
    r3 = Q1 * Q2 * a3 + Q1 * Q2 * Q3 * a4 + Q1 * Q2 * Q3 * Q4 * a5 + Q1 * Q2 * Q3 * Q4 * Q5 * a6
    r4 = Q1 * Q2 * Q3 * a4 + Q1 * Q2 * Q3 * Q4 * a5 + Q1 * Q2 * Q3 * Q4 * Q5 * a6
    r5 = Q1 * Q2 * Q3 * Q4 * a5 + Q1 * Q2 * Q3 * Q4 * Q5 * a6
    r6 = Q1 * Q2 * Q3 * Q4 * Q5 * a6

    E = np.matrix([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
    s1 = np.matrix([[1], [0], [0]])
    s2 = np.matrix([[0], [1], [0]])
    s3 = np.matrix([[0], [0], [1]])
    EQ1 = E * Q
    EQ2 = Q1 * E * Q2 * Q3 * Q4 * Q5 * Q6
    EQ3 = Q1 * Q2 * E * Q3 * Q4 * Q5 * Q6
    EQ4 = Q1 * Q2 * Q3 * E * Q4 * Q5 * Q6
    EQ5 = Q1 * Q2 * Q3 * Q4 * E * Q5 * Q6
    EQ6 = Q1 * Q2 * Q3 * Q4 * Q5 * E * Q6

    R1 = np.concatenate((EQ1 * s1, EQ2 * s1, EQ3 * s1, EQ4 * s1, EQ5 * s1, EQ6 * s1), axis=1)
    R2 = np.concatenate((EQ1 * s2, EQ2 * s2, EQ3 * s2, EQ4 * s2, EQ5 * s2, EQ6 * s2), axis=1)
    R3 = np.concatenate((EQ1 * s3, EQ2 * s3, EQ3 * s3, EQ4 * s3, EQ5 * s3, EQ6 * s3), axis=1)

    Jacobian_temp_1 = np.concatenate((R1, R2, R3), axis=0)

    Jacobian_temp_2 = np.column_stack((np.cross(e1.getA1(), r1.getA1()),
                                       np.cross(e2.getA1(), r2.getA1()),
                                       np.cross(e3.getA1(), r3.getA1()),
                                       np.cross(e4.getA1(), r4.getA1()),
                                       np.cross(e5.getA1(), r5.getA1()),
                                       np.cross(e6.getA1(), r6.getA1())))

    jacobian = np.matrix(np.concatenate((Jacobian_temp_1, Jacobian_temp_2), axis=0))
    return jacobian, Q, s1, s2, s3, r1


def jaco_inverse_kinematics(theta_current, pose_goal, Q_goal):

    if theta_current is not np.matrix:
        theta_current = np.matrix(theta_current)
    if pose_goal is not np.matrix:
        pose_goal = np.matrix(pose_goal)
    if Q_goal is not np.matrix:
        Q_goal = np.matrix(Q_goal)
    if theta_current.shape != (6, 1):
        theta_current = theta_current.transpose()
    theta_output = theta_current
    theta_offset = np.matrix([[0], [-np.pi / 2], [np.pi / 2], [2 * np.pi], [np.pi], [np.pi]])
    theta = theta_current

    # Parametres physiques du robot Jaco
    D1 = 0.2755
    D2 = 0.41
    D3 = 0.2073
    D4 = 0.0743
    D5 = 0.0743
    D6 = 0.1687
    E2 = 0.0098

    # Parametres Intermediaires
    aa = 11 * np.pi / 72
    ca = (np.cos(aa))
    sa = (np.sin(aa))
    c2a = (np.cos(2 * aa))
    s2a = (np.sin(2 * aa))
    d4b = (D3 + (ca - c2a / s2a * sa) * D4)
    d5b = (sa / s2a * D4 + (ca - c2a / s2a * sa) * D5)
    d6b = (sa / s2a * D5 + D6)

    a = np.matrix([0, D2, 0, 0, 0, 0])
    d = np.matrix([D1, 0, -E2, -d4b, -d5b, -d6b])
    alpha = np.matrix([np.pi / 2, np.pi, np.pi / 2, 2 * aa, 2 * aa, np.pi])

    # #default values
    # Q = np.eye(3)
    # s1 = np.matrix([[1], [0], [0]])
    # s2 = np.matrix([[0], [1], [0]])
    # s3 = np.matrix([[0], [0], [1]])
    # r1 = np.matrix([[1], [0], [0]])

    # %XXXXXXXXXXXXXXXXXXXXXX PGI XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    # --- PARAMETRES DE LA FONCTION-----------------------------------------------------
    sample_time = 0.005
    erreur = 0
    epsilon = 10 ** -4
    max_loops = 10
    success = False
    # --- BOUCLE PRINCIPALE ------------------------------------------------------------
    for loops in range(max_loops):
        # not the "usual" jacobian we normally use for classic inverse kinematic resolutions.
        jacobian, Q, s1, s2, s3, r1 = get_jaco_jacobian(theta, a, d, alpha)
        #--- PERFORMANCE ACTUELLE - --------------------------------------------------------
        f = np.concatenate(((Q - Q_goal) * s1, (Q - Q_goal) * s2, (Q - Q_goal) * s3, r1 - pose_goal[0:3, 0]), axis=0)
        stop = np.max(np.abs(f)) < epsilon
        if stop:
            #we converged
            success = True
            return theta_output + theta_offset, success


        # adapted damped jacobian to avoid singularities
        Ek = np.matrix(np.divide(f.transpose() * f, 2.0))
        Wn = np.matrix(np.multiply(np.eye(6), Ek + 0.001))
        Hk = np.matrix(jacobian.transpose() * jacobian + Wn)
        jacobian_inv = Hk.I * jacobian.transpose()
        d_theta = jacobian_inv * f
        theta_output = theta_output - d_theta
        theta = theta_output
    return theta_output + theta_offset, success


def jaco_direct_kinematics(theta_actuators):

    if theta_actuators is not np.matrix:
        theta_actuators = np.matrix(theta_actuators)
    if theta_actuators.shape != (6,1):
        theta_actuators = theta_actuators.transpose()
    theta_offset = np.matrix([0, -np.pi / 2, np.pi / 2, 2 * np.pi, np.pi, np.pi])
    theta_actuators = np.subtract(theta_actuators, theta_offset.transpose())
    # Parametres physiques du robot Jaco
    D1 = 0.2755
    D2 = 0.41
    D3 = 0.2073
    D4 = 0.0743
    D5 = 0.0743
    D6 = 0.1687
    E2 = 0.0098

    # Parametres Intermediaires
    aa = 11 * np.pi / 72
    ca = (np.cos(aa))
    sa = (np.sin(aa))
    c2a = (np.cos(2 * aa))
    s2a = (np.sin(2 * aa))
    d4b = (D3 + (ca - c2a / s2a * sa) * D4)
    d5b = (sa / s2a * D4 + (ca - c2a / s2a * sa) * D5)
    d6b = (sa / s2a * D5 + D6)

    a = np.matrix([0, D2, 0, 0, 0, 0])
    d = np.matrix([D1, 0, -E2, -d4b, -d5b, -d6b])
    alpha = np.matrix([np.pi / 2, np.pi, np.pi / 2, 2 * aa, 2 * aa, np.pi])
    theta1 = theta_actuators[0, 0]
    theta2 = theta_actuators[1, 0]
    theta3 = theta_actuators[2, 0]
    theta4 = theta_actuators[3, 0]
    theta5 = theta_actuators[4, 0]
    theta6 = theta_actuators[5, 0]

    a1 = np.matrix([[a[0, 0] * np.cos(theta1)],
                    [a[0, 0] * np.sin(theta1)],
                    [d[0, 0]]])
    a2 = np.matrix([[a[0, 1] * np.cos(theta2)],
                    [a[0, 1] * np.sin(theta2)],
                    [d[0, 1]]])
    a3 = np.matrix([[a[0, 2] * np.cos(theta3)],
                    [a[0, 2] * np.sin(theta3)],
                    [d[0, 2]]])
    a4 = np.matrix([[a[0, 3] * np.cos(theta4)],
                    [a[0, 3] * np.sin(theta4)],
                    [d[0, 3]]])
    a5 = np.matrix([[a[0, 4] * np.cos(theta5)],
                    [a[0, 4] * np.sin(theta5)],
                    [d[0, 4]]])
    a6 = np.matrix([[a[0, 5] * np.cos(theta6)],
                    [a[0, 5] * np.sin(theta6)],
                    [d[0, 5]]])

    Q1 = np.matrix([[np.cos(theta1), -np.cos(alpha[0, 0]) * np.sin(theta1), np.sin(alpha[0, 0]) * np.sin(theta1)],
                    [np.sin(theta1), np.cos(alpha[0, 0]) * np.cos(theta1), -np.sin(alpha[0, 0]) * np.cos(theta1)],
                    [0, np.sin(alpha[0, 0]), np.cos(alpha[0, 0])]])
    Q2 = np.matrix([[np.cos(theta2), -np.cos(alpha[0, 1]) * np.sin(theta2), np.sin(alpha[0, 1]) * np.sin(theta2)],
                    [np.sin(theta2), np.cos(alpha[0, 1]) * np.cos(theta2), -np.sin(alpha[0, 1]) * np.cos(theta2)],
                    [0, np.sin(alpha[0, 1]), np.cos(alpha[0, 1])]])
    Q3 = np.matrix([[np.cos(theta3), -np.cos(alpha[0, 2]) * np.sin(theta3), np.sin(alpha[0, 2]) * np.sin(theta3)],
                    [np.sin(theta3), np.cos(alpha[0, 2]) * np.cos(theta3), -np.sin(alpha[0, 2]) * np.cos(theta3)],
                    [0, np.sin(alpha[0, 2]), np.cos(alpha[0, 2])]])
    Q4 = np.matrix([[np.cos(theta4), -np.cos(alpha[0, 3]) * np.sin(theta4), np.sin(alpha[0, 3]) * np.sin(theta4)],
                    [np.sin(theta4), np.cos(alpha[0, 3]) * np.cos(theta4), -np.sin(alpha[0, 3]) * np.cos(theta4)],
                    [0, np.sin(alpha[0, 3]), np.cos(alpha[0, 3])]])
    Q5 = np.matrix([[np.cos(theta5), -np.cos(alpha[0, 4]) * np.sin(theta5), np.sin(alpha[0, 4]) * np.sin(theta5)],
                    [np.sin(theta5), np.cos(alpha[0, 4]) * np.cos(theta5), -np.sin(alpha[0, 4]) * np.cos(theta5)],
                    [0, np.sin(alpha[0, 4]), np.cos(alpha[0, 4])]])
    Q6 = np.matrix([[np.cos(theta6), -np.cos(alpha[0, 5]) * np.sin(theta6), np.sin(alpha[0, 5]) * np.sin(theta6)],
                    [np.sin(theta6), np.cos(alpha[0, 5]) * np.cos(theta6), -np.sin(alpha[0, 5]) * np.cos(theta6)],
                    [0, np.sin(alpha[0, 5]), np.cos(alpha[0, 5])]])
    Q = Q1 * Q2 * Q3 * Q4 * Q5 * Q6

    cartesian_pose = a1 + Q1 * a2 + Q1 * Q2 * a3 + Q1 * Q2 * Q3 * a4 + Q1 * Q2 * Q3 * Q4 * a5 + Q1 * Q2 * Q3 * Q4 * Q5 * a6

    return cartesian_pose, Q


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
    theta = np.array([2, 2, 2, 2, 2, 2])
    old_pose, old_Q = jaco_direct_kinematics(theta)
    print(old_pose)
    theta, success = jaco_inverse_kinematics(theta, old_pose, old_Q)
    theta = (theta + np.pi) % (2 * np.pi) - np.pi

    print(theta, success)
    old_pose, old_Q = jaco_direct_kinematics(theta)
    theta, success = jaco_inverse_kinematics(theta, old_pose, old_Q)
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    print(theta, success)
    old_pose, old_Q = jaco_direct_kinematics(theta)
    theta, success = jaco_inverse_kinematics(theta, old_pose, old_Q)
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    print(theta, success)
    old_pose, old_Q = jaco_direct_kinematics(theta)
    theta, success = jaco_inverse_kinematics(theta, old_pose, old_Q)
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    print(theta, success)
    old_pose, old_Q = jaco_direct_kinematics(theta)
    theta, success = jaco_inverse_kinematics(theta, old_pose, old_Q)
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    print(theta, success)
    # print("direct kinematics")
    # print(Q)
    # print("euleur angles")
    # euler = rotationMatrixToEulerAngles(Q)
    # print(euler)
    # print("Q recomposee avec euler angles")
    # new_Q = eulerAnglesToRotationMatrix(euler)
    # print(new_Q)
    # print("difference entre les deux")
    # print(Q - new_Q)
    # for i in range(100):
    #     pose, Q = jaco_direct_kinematics(theta)
    #
    #     angular_velocities = convert_to_angular_velocities([-0.01, 0, 0, 0, 0, 0], theta, 0.01)
    #     angular_velocities = angular_velocities.reshape(1, 6)[0]
    #     # print(angular_velocities)
    #     # print(theta)
    #     theta += angular_velocities
    #     # print(theta)
    #     print(np.subtract(pose, old_pose) / 0.1)
    #     old_pose = pose
    # for i in range(100):
    #     pose, Q = jaco_direct_kinematics(theta)
    #     angular_velocities = convert_to_angular_velocities([0.01, 0, 0, 0, 0, 0], theta, 0.01)
    #     angular_velocities = angular_velocities.reshape(1, 6)[0]
    #     #print(angular_velocities)
    #     #print(theta)
    #     theta += angular_velocities
    #     #print(theta)
    #     #print(angular_velocities)
    #     print(np.subtract(pose, old_pose) / 10)
    #     old_pose = pose








