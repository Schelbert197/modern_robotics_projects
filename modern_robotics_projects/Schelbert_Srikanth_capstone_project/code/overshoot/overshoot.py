import modern_robotics as mr
import numpy as np
import csv
import matplotlib.pyplot as plt

Blist = np.array([[0,         0,         1,         0,    0.033,        0],
                  [0,        -1,         0,   -0.5076,        0,        0],
                  [0,        -1,         0,   -0.3526,        0,        0],
                  [0,        -1,         0,   -0.2176,        0,        0],
                  [0,         0,         1,         0,        0,        0]]).T

T_sci = np.array([[1, 0, 0, 1],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])

T_scf = np.array([[0, 1, 0, 0],
                  [-1, 0, 0, -1],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])

T_b0 = np.array([[1, 0, 0, 0.1662],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0.0026],
                 [0, 0, 0, 1]])

M_0e = np.array([[1, 0, 0, 0.033],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0.6546],
                 [0, 0, 0, 1]])

T_sei = np.array([[0, 0, 1, 0],
                  [0, 1, 0, 0],
                  [-1, 0, 0, 0.5],
                  [0, 0, 0, 1]])

T_ces = np.array([[-1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, -1, 0.2],
                  [0, 0, 0, 1]])  # ce standoff

T_cegrasp = np.array([[-1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, -1, 0],
                      [0, 0, 0, 1]])

k = 1

identity6 = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])

Ki = 10.50*identity6  # 10.5
Kp = 1.0*identity6
dt = 0.01

wheel_radius = 0.0475
l_wheel = 0.47/2
w_wheel = 0.15
gamma_13 = -np.pi/4
gamma_24 = np.pi/4

F_pseudo = np.array([[-1/(l_wheel+w_wheel), 1/(l_wheel+w_wheel),
                    1/(l_wheel+w_wheel), -1/(l_wheel+w_wheel)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])*(wheel_radius/4)

F6 = np.array([[0, 0, 0, 0], [0, 0, 0, 0], F_pseudo[0],
              F_pseudo[1], F_pseudo[2], [0, 0, 0, 0]])


def TrajectoryGenerator(T_sei, T_sci, T_scf, T_cegrasp, T_cestandoff, k):
    """
    Get the desired trajectory.

    Generates the reference trajectory for the end effector frame.

    Args:
    ----
    Tsei: Initial Configuration of the EE in the space frame.
    Tsci: Initial Configuration of the cube in the space frame.
    Tscf: Final Configuration of the cube in the space frame.
    Tcegrasp: EE configuration relative to the cube when picked up.
    Tcestandoff: EE standoff configuration relative to cube.
    k: Number of trajectory reference configurations per 0.01 seconds.

    Returns
    -------
    config_list: List of 13-vector configurations of the end-effector
    """

    T_ees1 = T_sci@T_cestandoff
    config_list = []

    # Trajectory to standoff above cube
    traj1 = mr.CartesianTrajectory(T_sei, T_ees1, 4, 4*k/0.01, 3)
    for i in range(0, len(traj1)):
        config_line = []
        for j in range(0, 3):
            for k in range(0, 3):
                config_line.append(traj1[i][j][k])
        for q in range(0, 3):
            config_line.append(traj1[i][q][3])
        config_line.append(0)
        config_list.append(config_line)

    T_eegrasp1 = T_sci@T_cegrasp

    # Trajectory to cube
    traj2 = mr.CartesianTrajectory(T_ees1, T_eegrasp1, 4, 4*k/0.01, 3)
    for i in range(0, len(traj2)):
        config_line = []
        for j in range(0, 3):
            for k in range(0, 3):
                config_line.append(traj2[i][j][k])
        for q in range(0, 3):
            config_line.append(traj2[i][q][3])
        config_line.append(0)
        config_list.append(config_line)

    # Trajectory grab cube
    traj3 = mr.CartesianTrajectory(T_eegrasp1, T_eegrasp1, 1, 1*k/0.01, 3)
    for i in range(0, len(traj3)):
        config_line = []
        for j in range(0, 3):
            for k in range(0, 3):
                config_line.append(traj3[i][j][k])
        for q in range(0, 3):
            config_line.append(traj3[i][q][3])
        config_line.append(1)
        config_list.append(config_line)

    # Trajectory to move back to standoff
    traj4 = mr.CartesianTrajectory(T_eegrasp1, T_ees1, 4, 4*k/0.01, 3)
    for i in range(0, len(traj4)):
        config_line = []
        for j in range(0, 3):
            for k in range(0, 3):
                config_line.append(traj4[i][j][k])
        for q in range(0, 3):
            config_line.append(traj4[i][q][3])
        config_line.append(1)
        config_list.append(config_line)

    T_ees2 = T_scf@T_cestandoff

    # Trajectory to move to standoff 2
    traj5 = mr.ScrewTrajectory(T_ees1, T_ees2, 6, 6*k/0.01, 3)
    for i in range(0, len(traj5)):
        config_line = []
        for j in range(0, 3):
            for k in range(0, 3):
                config_line.append(traj5[i][j][k])
        for q in range(0, 3):
            config_line.append(traj5[i][q][3])
        config_line.append(1)
        config_list.append(config_line)

    T_eegrasp2 = T_scf@T_cegrasp

    # Trajectory to move to final block pos
    traj6 = mr.CartesianTrajectory(T_ees2, T_eegrasp2, 6, 6*k/0.01, 3)
    for i in range(0, len(traj6)):
        config_line = []
        for j in range(0, 3):
            for k in range(0, 3):
                config_line.append(traj6[i][j][k])
        for q in range(0, 3):
            config_line.append(traj6[i][q][3])
        config_line.append(1)
        config_list.append(config_line)

    # Trajectory to ungrasp block
    traj7 = mr.CartesianTrajectory(T_eegrasp2, T_eegrasp2, 1, 1*k/0.01, 3)
    for i in range(0, len(traj7)):
        config_line = []
        for j in range(0, 3):
            for k in range(0, 3):
                config_line.append(traj7[i][j][k])
        for q in range(0, 3):
            config_line.append(traj7[i][q][3])
        config_line.append(0)
        config_list.append(config_line)

    # Trajectory to ungrasp block
    traj8 = mr.CartesianTrajectory(T_eegrasp2, T_ees2, 3, 3*k/0.01, 3)
    for i in range(0, len(traj8)):
        config_line = []
        for j in range(0, 3):
            for k in range(0, 3):
                config_line.append(traj8[i][j][k])
        for q in range(0, 3):
            config_line.append(traj8[i][q][3])
        config_line.append(0)
        config_list.append(config_line)

    # with open('traj.csv', 'w', newline='') as f:
    #     writer = csv.writer(f)
    #     writer.writerows(config_list)

    return config_list


def NextState(config, velocities, dt, w_max):
    """
    Get the next configuration state.

    Computes the configuration at the next timestep.

    Args:
    ----
    config: A configuration 13-vector for the chassis, arm joints,
    wheels, and gripper state.
    velocities: A 9-vector containing the wheel arm joint omegas (rad/s).
    dt: Timestep in seconds

    Returns
    -------
    new_config: A new configuration 13-vector for the next timestep.
    """

    # config: chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5,
    # W1, W2, W3, W4, gripper state
    # velocities: 4 u vars, 5 arm joint speeds
    new_config = config

    # Cap the limit of the joint velocities
    for i in range(0, len(velocities)):
        if velocities[i] > w_max:
            velocities[i] = w_max
        elif velocities[i] < -w_max:
            velocities[i] = -w_max

    phi, x, y = config[0], config[1], config[2]

    u_vals = velocities[:4]
    Vb = F_pseudo @ u_vals
    # Vb6 = [0, 0, Vb[0], Vb[1], Vb[2], 0]

    if 0.0001 > Vb[0] > -0.0001:
        Vb = Vb
    else:
        Vb[0] = Vb[0]
        Vb[1] = ((Vb[1]*np.sin(Vb[0])) + (Vb[2]*(np.cos(Vb[0])-1)))/Vb[0]
        Vb[2] = ((Vb[2]*np.sin(Vb[0])) + (Vb[1]*(1-np.cos(Vb[0]))))/Vb[0]

    q_rot = np.array([[1, 0, 0],
                      [0, np.cos(phi), -np.sin(phi)],
                      [0, np.sin(phi), np.cos(phi)]])

    # This rotation takes the body twist and trasfers it back to world frame
    Vb = q_rot@Vb

    for i in range(3, 8):
        new_config[i] = config[i] + (velocities[i+1] * dt)

    for j in range(8, 12):
        new_config[j] = config[j] + (velocities[j-8] * dt)

    new_config[0] = phi + (Vb[0] * dt)
    new_config[1] = x + (Vb[1] * dt)
    new_config[2] = y + (Vb[2] * dt)

    return new_config


def GetCurrentXJacobian(config, Tb0, M_0e, Blist):
    """
    Calculate X and Jacobian.

    Helper function for getting the current X matrix and Jacobian.
    Args:
    ----
    config: A configuration 13-vector for the chassis, arm joints, wheels,
    and gripper state.
    Tb0: The transformation matrix from the chassis origin to the arm base.
    M_0e: The M matrix when the arm is at the home configuration.
    Blist: The body screw axes when the robot is in the home configuration.

    Returns
    -------
    Tse: A new EE configuration matrix with updated positions.
    J_pseudo: The pseudoinverse of the body jacobian for the whole robot
    """

    # X = Tse = Tsb_q @ Tb0 T0e
    phi, x, y = config[0], config[1], config[2]

    J1, J2, J3, J4, J5 = config[3], config[4], config[5], config[6], config[7]
    thetalist_t = [J1, J2, J3, J4, J5]

    T_sb_q = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                       [np.sin(phi), np.cos(phi), 0, y],
                       [0, 0, 1, 0.0963],
                       [0, 0, 0, 1]])
    T_0e = mr.FKinBody(M_0e, Blist, thetalist_t)

    Tse = T_sb_q @ Tb0 @ T_0e

    J_base = mr.Adjoint(mr.TransInv(Tse)@T_sb_q)@F6

    J_arm = mr.JacobianBody(Blist, thetalist_t)
    # if the arm joints are too close to collision (summing to pi),
    # we need to set the corresponding columns of J_arm to 0

    # if abs(J3+J4) >= np.pi:
    #     J_arm[:, 2] = np.zeros(6)
    #     J_arm[:, 3] = np.zeros(6)

    Je = np.hstack((J_base, J_arm))

    # If tall
    # J_pseduo = np.linalg.inv(Je.T@Je)@Je.T
    # If wide
    J_pseudo = Je.T@np.linalg.inv(Je@Je.T)

    return Tse, J_pseudo


def FeedbackControl(Tse, Tse_d, Tse_d_next, Kp, Ki, dt):
    """
    Controller.

    Feedforward and feedback control of the robot.

    Args:
    ----
    Tse: The current configuration matrix of the EE in space
    Tse_d: The desired configuration matrix of the EE in space
    Tse_d_next: The next desired configuration matrix of the EE in space
    Kp: The proportional gain matrix of the controller
    Ki: The integral gain matrix of the controller

    Returns
    -------
    V: The twist of the robot
    """

    # Invert the current configuration matrix
    X_inv = mr.TransInv(Tse)

    # Compute the error twist of the configuration
    X_err_twist = mr.se3ToVec(mr.MatrixLog6(X_inv@Tse_d))

    # Calculate twist vectors for the V(t) equation
    Vd = mr.se3ToVec((1/dt)*mr.MatrixLog6(mr.TransInv(Tse_d)@Tse_d_next))
    Vb = mr.Adjoint(X_inv@Tse_d)@Vd

    V = Vb + Kp@X_err_twist + Ki@(X_err_twist + (dt * X_err_twist))

    return V, X_err_twist


def GetVelocities(J_pseudo, V):
    """
    Get the velocities.

    Calculates the next set of velocities for the robot.

    Args:
    ----
    J_pseudo: The pseudoinverse of the jacobian of the robot.
    V: The twist of the robot.

    Returns
    -------
    u_thetadot: The velocities of the wheels and the arm joints
    """

    u_thetadot = J_pseudo@V
    return u_thetadot


def TrajtoMatrix(Traj):
    """
    Trajectory to matrix.

    Takes the trajectory 13-array and returns a matrix.

    Args:
    ----
    Trajectory: A 13-vector for the EE including R, p, and the gripper state

    Returns
    -------
    T_mat: A trajectory point as a transformation matrix without gripper state.
    """

    T_mat = np.array([[Traj[0], Traj[1], Traj[2], Traj[9]],
                      [Traj[3], Traj[4], Traj[5], Traj[10]],
                      [Traj[6], Traj[7], Traj[8], Traj[11]],
                      [0, 0, 0, 1]])
    return T_mat

# Trying the full task


# Generate the desired trajectory
print("Begin Script")
EE_traj_desired = TrajectoryGenerator(T_sei, T_sci, T_scf, T_cegrasp, T_ces, k)
print("Goal Trajectory Created")
current_config = [0, 0, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0]
velocities = []
bot_config_list = []
bot_config_list.append(current_config)

with open('new_traj_plot.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(current_config)

    X_errors = []
    times = []

    for i in range(0, len(EE_traj_desired)-1):
        # Calculates the current EE configuration and pseudoinverse Jacobian
        Tse_current, J_pseudo_current = GetCurrentXJacobian(
            current_config, T_b0, M_0e, Blist)

        # Calculates the desired EE position from the generated trajectory
        T_se_desired = TrajtoMatrix(EE_traj_desired[i])

        # Calculates the next desired EE position from the generated trajectory
        T_se_desired_next = TrajtoMatrix(EE_traj_desired[i+1])

        # Calculates the twist and the error twist of the robot at the timestep
        V_twist, X_err_current = FeedbackControl(
            Tse_current, T_se_desired, T_se_desired_next, Kp, Ki, dt)

        # Creating the lists to plot the error as the function runs
        X_errors.append(X_err_current)
        times.append(i*0.01)

        # Calculates the velocities for the wheels and the arm joints
        velocities = GetVelocities(J_pseudo_current, V_twist)

        # Calculates the next configuration for the next timestep
        new_configuration = NextState(current_config, velocities, dt, 10)

        # Establishes the current configuration for the next loop
        current_config = new_configuration

        # Ensures that the gripper state is maintained
        current_config[-1] = EE_traj_desired[i][-1]

        # Append the list of current configurations
        bot_config_list.append(current_config)
        # print(current_config[1])
        writer.writerow(current_config)

    fig = plt.figure()

    ax = fig.add_subplot()

    X_errors = np.array(X_errors)
    ax.plot(times, X_errors[:, 0], c='r', label='Omega x error (rad/s)')
    ax.plot(times, X_errors[:, 1], c='g', label='Omega y error (rad/s)')
    ax.plot(times, X_errors[:, 2], c='b', label='Omega z error (rad/s)')
    ax.plot(times, X_errors[:, 3], c='y', label='Linear x error (m/s)')
    ax.plot(times, X_errors[:, 4], c='orange', label='Linear y error (m/s)')
    ax.plot(times, X_errors[:, 5], c='purple', label='Linear z error (m/s)')
    plt.legend()
    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Error')
    plt.show()

print("Generating Plot")
with open('X_error_overshoot.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(X_errors)
print("Done")
