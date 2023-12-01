import modern_robotics as mr
import numpy as np
import math
import csv

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
                  [0, 0, 0, 1]]) # ce standoff

T_cegrasp = np.array([[-1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])

k = 1

identity = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

Ki = 0.0*identity
Kp = 0.0*identity


wheel_radius = 0.0475
l_wheel = 0.47/2
w_wheel = 0.15
gamma_13 = -np.pi/4
gamma_24 = np.pi/4

F_pseduo = np.array([[-1/(l_wheel+w_wheel), 1/(l_wheel+w_wheel), 1/(l_wheel+w_wheel), -1/(l_wheel+w_wheel)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])*(wheel_radius/4)

def TrajectoryGenerator(T_sei, T_sci, T_scf, T_cegrasp, T_cestandoff, k):
    """Generates the reference trajectory for the end effector frame"""

    T_ees1 = T_sci@T_cestandoff
    csv_list = []
    
    # Trajectory to standoff above cube
    traj1 = mr.CartesianTrajectory(T_sei,T_ees1,4,4*k/0.01,3)
    for i in range(0,len(traj1)):
        csv_line = []
        for j in range(0,3):
            for l in range(0,3):
                csv_line.append(traj1[i][j][l])
        for q in range(0,3):
            csv_line.append(traj1[i][q][3])
        csv_line.append(0)
        csv_list.append(csv_line)

    T_eegrasp1 = T_sci@T_cegrasp

    # Trajectory to cube
    traj2 = mr.CartesianTrajectory(T_ees1,T_eegrasp1,4,4*k/0.01,3)
    for i in range(0,len(traj2)):
        csv_line = []
        for j in range(0,3):
            for l in range(0,3):
                csv_line.append(traj2[i][j][l])
        for q in range(0,3):
            csv_line.append(traj2[i][q][3])
        csv_line.append(0)
        csv_list.append(csv_line)

    # Trajectory grab cube
    traj3 = mr.CartesianTrajectory(T_eegrasp1,T_eegrasp1,1,1*k/0.01,3)
    for i in range(0,len(traj3)):
        csv_line = []
        for j in range(0,3):
            for l in range(0,3):
                csv_line.append(traj3[i][j][l])
        for q in range(0,3):
            csv_line.append(traj3[i][q][3])
        csv_line.append(1)
        csv_list.append(csv_line)

    # Trajectory to move back to standoff
    traj4 = mr.CartesianTrajectory(T_eegrasp1,T_ees1,4,4*k/0.01,3)
    for i in range(0,len(traj4)):
        csv_line = []
        for j in range(0,3):
            for l in range(0,3):
                csv_line.append(traj4[i][j][l])
        for q in range(0,3):
            csv_line.append(traj4[i][q][3])
        csv_line.append(1)
        csv_list.append(csv_line)

    T_ees2 = T_scf@T_cestandoff

    # Trajectory to move to standoff 2
    traj5 = mr.ScrewTrajectory(T_ees1,T_ees2,6,6*k/0.01,3)
    for i in range(0,len(traj5)):
        csv_line = []
        for j in range(0,3):
            for l in range(0,3):
                csv_line.append(traj5[i][j][l])
        for q in range(0,3):
            csv_line.append(traj5[i][q][3])
        csv_line.append(1)
        csv_list.append(csv_line)

    T_eegrasp2 = T_scf@T_cegrasp

    # Trajectory to move to final block pos
    traj6 = mr.CartesianTrajectory(T_ees2,T_eegrasp2,6,6*k/0.01,3)
    for i in range(0,len(traj6)):
        csv_line = []
        for j in range(0,3):
            for l in range(0,3):
                csv_line.append(traj6[i][j][l])
        for q in range(0,3):
            csv_line.append(traj6[i][q][3])
        csv_line.append(1)
        csv_list.append(csv_line)

    # Trajectory to ungrasp block
    traj7 = mr.CartesianTrajectory(T_eegrasp2,T_eegrasp2,1,1*k/0.01,3)
    for i in range(0,len(traj7)):
        csv_line = []
        for j in range(0,3):
            for l in range(0,3):
                csv_line.append(traj7[i][j][l])
        for q in range(0,3):
            csv_line.append(traj7[i][q][3])
        csv_line.append(0)
        csv_list.append(csv_line)

    # Trajectory to ungrasp block
    traj8 = mr.CartesianTrajectory(T_eegrasp2,T_ees2,3,3*k/0.01,3)
    for i in range(0,len(traj8)):
        csv_line = []
        for j in range(0,3):
            for l in range(0,3):
                csv_line.append(traj8[i][j][l])
        for q in range(0,3):
            csv_line.append(traj8[i][q][3])
        csv_line.append(0)
        csv_list.append(csv_line)

    with open('traj.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(csv_list)

def NextState(config, velocities, dt, w_max):
    """Function for milestone 1"""
    # config: chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state
    # velocities: 4 u vars, 5 arm joint speeds
    new_config = config

    phi, x, y = config[0], config[1], config[2]
    
    
    u_vals = velocities[:4]
    Vb = F_pseduo @ u_vals
    Vb6 = [0, 0, Vb[0], Vb[1], Vb[2], 0]

    for i in range(3,8):
        new_config[i] = config[i] + (velocities[i+1] * dt)

    for j in range(8,12):
        new_config[j] = config[j] + (velocities[j-8] * dt)

    new_config [0] = config[0] + (Vb[0] * dt)
    new_config [1] = config[1] + (Vb[1] * dt)
    new_config [2] = config[2] + (Vb[2] * dt)

    return new_config
    # Tse = T_sb_q @ T_b0 @ T_0e

    # new arm joint theta = old arm joint angles + (joint speeds * dt)
    # new wheel angles = old wheel joint angles + (wheels speeds * dt)
    # new chassis config obtained from odometry

    # Return a new config

################# FOR PART 2 ###################
# TrajectoryGenerator(T_sei, T_sci, T_scf, T_cegrasp, T_ces, k)

config = [0,0,0,0,0,0,0,0,0,0,0,0,0]
velocities = [-10,10,-10,10,0,0,0,0,0]
csv_list = []
for i in range(0,100):
    csv_line = []
    next_c = NextState(config, velocities, 0.01, 10)
    for j in range(0,len(next_c)):
        csv_line.append(next_c[j])
    csv_list.append(csv_line)

with open('traj.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(csv_list)

    

# T_sb_q = np.array([[np.cos(phi), -np.sin(phi), 0, x],
#                     [np.sin(phi), np.cos(phi), 0, y],
#                     [0, 0, 1, 0.0963],
#                     [0, 0, 0, 1]])

# u_vals = velocities[:4]
# Vb = F_pseduo @ u_vals
# Vb6 = [0, 0, Vb[0], Vb[1], Vb[2], 0]

# mr.MatrixExp6(mr.VecTose3(Vb6))



def get_current_X(config, Tb0, M_0e, Blist):
    """Helper function for getting the current X matrix"""
    # X = Tse = Tsb_q @ Tb0 T0e
    phi, x, y = config[0], config[1], config[2]
    
    J1,J2,J3,J4,J5 = config[3], config[4], config[5], config[6], config[7]
    thetalist_t = [J1,J2,J3,J4,J5]

    T_sb_q = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                        [np.sin(phi), np.cos(phi), 0, y],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])
    T_0e = mr.FKinBody(M_0e, Blist, thetalist_t)

    Tse = T_sb_q @ Tb0 @ T_0e

    return Tse

def FeedbackControl(Tse, Tse_d, Tse_d_next, Kp, Ki, dt):
    """Feedforward and feedback control"""
    X_inv = mr.TransInv(Tse)
    # X_inv = np.linalg.inv(Tse) other option
    X_err_twist = mr.se3ToVec(mr.MatrixLog6(X_inv@Tse_d))

    Vd = mr.se3ToVec((1/dt)*mr.MatrixLog6(Tse_d@Tse_d_next))
    Vb = mr.Adjoint(X_inv@Tse_d)@Vd

    V = Vb + Kp@X_err_twist + Ki@(X_err_twist + (dt * X_err_twist))
    
    return V

Xd = np.array([[0, 0, 1, 0.5],
                [0, 1, 0, 0],
                [-1, 0, 0, 0.5],
                [0, 0, 0, 1]])

Xd_next = np.array([[0, 0, 1, 0.6],
                    [0, 1, 0, 0],
                    [-1, 0, 0, 0.3],
                    [0, 0, 0, 1]])

m3_config = [0,0,0,0,0,0.2,-1.6,0]

print(get_current_X(m3_config, T_b0, M_0e, Blist))