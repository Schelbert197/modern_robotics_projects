import modern_robotics as mr
import numpy as np
import csv


T_sci = np.array([[1, 0, 0, 1],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0.025],
                  [0, 0, 0, 1]])

T_scf = np.array([[0, 1, 0, 0],
                  [-1, 0, 0, 1],
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

    with open('new_traj.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(config_list)

    # return config_list


# FOR PART 2
TrajectoryGenerator(T_sei, T_sci, T_scf, T_cegrasp, T_ces, k)
