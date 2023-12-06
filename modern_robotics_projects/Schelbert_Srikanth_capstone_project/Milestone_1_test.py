import numpy as np
import csv

wheel_radius = 0.0475
l_wheel = 0.47/2
w_wheel = 0.15
F_pseudo = np.array([[-1/(l_wheel+w_wheel), 1/(l_wheel+w_wheel),
                    1/(l_wheel+w_wheel), -1/(l_wheel+w_wheel)],
                    [1, 1, 1, 1],
                    [-1, 1, -1, 1]])*(wheel_radius/4)


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


# MILESTONE 1 TESTING
config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
velocities = [-10, 10, -10, 10, 0, 0, 0, 0, 0]
csv_list = []
for i in range(0, 100):
    csv_line = []
    next_c = NextState(config, velocities, 0.01, 10)
    for j in range(0, len(next_c)):
        csv_line.append(next_c[j])
    csv_list.append(csv_line)

with open('traj.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(csv_list)
