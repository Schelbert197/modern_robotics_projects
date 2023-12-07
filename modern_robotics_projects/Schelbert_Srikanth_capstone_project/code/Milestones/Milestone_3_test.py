import modern_robotics as mr
import numpy as np

Xd = np.array([[0, 0, 1, 0.5],
               [0, 1, 0, 0],
               [-1, 0, 0, 0.5],
               [0, 0, 0, 1]])

Xd_next = np.array([[0, 0, 1, 0.6],
                    [0, 1, 0, 0],
                    [-1, 0, 0, 0.3],
                    [0, 0, 0, 1]])

m3_config = [0, 0, 0, 0, 0, 0.2, -1.6, 0]

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

identity6 = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])

Ki = 10.50*identity6
Kp = 1.0*identity6
dt = 0.01
wheel_radius = 0.0475
l_wheel = 0.47/2
w_wheel = 0.15

F_pseudo = np.array([[-1/(l_wheel+w_wheel), 1/(l_wheel+w_wheel),
                    1/(l_wheel+w_wheel), -1/(l_wheel+w_wheel)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])*(wheel_radius/4)

F6 = np.array([[0, 0, 0, 0], [0, 0, 0, 0], F_pseudo[0],
              F_pseudo[1], F_pseudo[2], [0, 0, 0, 0]])


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
    Je = np.hstack((J_base, J_arm))

    # If tall
    # J_pseduo = np.linalg.inv(Je.T@Je)@Je.T
    # If wide
    J_pseudo = Je.T@np.linalg.inv(Je@Je.T)

    return Tse, J_pseudo


Tse_test, J_pseudo_test = GetCurrentXJacobian(m3_config, T_b0, M_0e, Blist)

V_test = FeedbackControl(Tse_test, Xd, Xd_next, Kp, Ki, 0.01)
print(V_test)
print(GetVelocities(J_pseudo_test, V_test))


thetalist_j = [0, 0, 0.2, -1.6, 0]
J_arm_test = mr.JacobianBody(Blist, thetalist_j)
Je = np.hstack((J_base_test, J_arm_test))

# If tall
# J_pseduo = np.linalg.inv(Je.T@Je)@Je.T
# If wide
J_pseduo = Je.T@np.linalg.inv(Je@Je.T)

print(Je)
u_theta = J_pseduo@V_test
print(u_theta)
# Tse = T_sb_q @ T_b0 @ T_0e

# new arm joint theta = old arm joint angles + (joint speeds * dt)
# new wheel angles = old wheel joint angles + (wheels speeds * dt)
# new chassis config obtained from odometry

# Return a new config


################# MILESTONE 3 TESTING ############
# T_sb_q = np.array([[np.cos(phi), -np.sin(phi), 0, x],
#                     [np.sin(phi), np.cos(phi), 0, y],
#                     [0, 0, 1, 0.0963],
#                     [0, 0, 0, 1]])

# u_vals = velocities[:4]
# Vb = F_pseduo @ u_vals
# Vb6 = [0, 0, Vb[0], Vb[1], Vb[2], 0]

# mr.MatrixExp6(mr.VecTose3(Vb6))
