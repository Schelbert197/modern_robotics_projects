"""
To use this file and produce a trajectory simply run this file.

If you prefer to define the function yourself, simply use the
call below with the given matrices and the traj.csv file will
be produced:

TrajectoryGenerator(T_sei, T_sci, T_scf, T_cegrasp, T_ces, k)

"""

import modern_robotics as mr
import numpy as np
import math
import csv

Blist = [[0,         0,         1,         0,    0.033,        0],
         [0,        -1,         0,   -0.5076,        0,        0],
         [0,        -1,         0,   -0.3526,        0,        0],
         [0,        -1,         0,   -0.2176,        0,        0],
         [0,         0,         1,         0,        0,        0]]

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


TrajectoryGenerator(T_sei, T_sci, T_scf, T_cegrasp, T_ces, k)