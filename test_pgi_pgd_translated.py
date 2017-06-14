import numpy as np
from direct_kinematics_jaco import direct_kinematics_jaco
from inverse_kinematics_jaco import inverse_kinematics_jaco
from joint_states_publisher import convert_to_angular_velocities

if __name__ == '__main__':
    theta = np.array([[0.344],
                      [0.8],
                      [-1.275],
                      [0.711],
                      [-0.04],
                      [-0.688]])
    theta = np.array([[0.],
                      [10.*np.pi/180.],
                      [0.],
                      [0.],
                      [0.],
                      [0.]])
    #theta -= np.array([[0.], [-np.pi / 2.0], [np.pi / 2.0], [0.], [-np.pi], [5.0 * np.pi / 9.0]])

    old_pose, old_Q = direct_kinematics_jaco(theta)
    print(old_pose)
    theta, success, theta_approx = inverse_kinematics_jaco(old_pose, old_Q, theta)
    # theta = (theta + np.pi) % (2 * np.pi) - np.pi
    print(theta, success)
    # old_pose, old_Q = PGDVince(theta)
    # theta, success, theta_approx = PGIVince(old_pose, old_Q, theta)
    # # theta = (theta + np.pi) % (2 * np.pi) - np.pi
    # print(theta, success)
    # old_pose, old_Q = PGDVince(theta)
    # theta, success, theta_approx = PGIVince(old_pose, old_Q, theta)
    # # theta = (theta + np.pi) % (2 * np.pi) - np.pi
    # print(theta, success)
    # old_pose, old_Q = PGDVince(theta)
    # theta, success, theta_approx = PGIVince(old_pose, old_Q, theta)
    # # theta = (theta + np.pi) % (2 * np.pi) - np.pi
    # print(theta, success)
    # old_pose, old_Q = PGDVince(theta)
    # theta, success, theta_approx = PGIVince(old_pose, old_Q, theta)
    # # theta = (theta + np.pi) % (2 * np.pi) - np.pi
    # print(theta, success)
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
    #theta += np.array([[0.], [-np.pi / 2.0], [np.pi / 2.0], [0.], [-np.pi], [5.0 * np.pi / 9.0]])
    pose, Q = direct_kinematics_jaco(theta)
    print(pose)
    theta, success, theta_approx = inverse_kinematics_jaco(pose, Q, theta)
    print(theta, success)
    for i in range(10):
        # theta = (theta + np.pi) % (2 * np.pi) - np.pi
        angular_velocities = convert_to_angular_velocities([1, 0, 0, 0, 0, 0], theta, 0.001)
        # angular_velocities = angular_velocities.reshape(1, 6)[0]
        # print(angular_velocities)
        theta += np.array(angular_velocities) * 0.001
        # print(theta)
        pose, Q = direct_kinematics_jaco(theta)
        print(np.subtract(pose, old_pose) / 0.001)
        old_pose = pose
    # # for i in range(100):
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