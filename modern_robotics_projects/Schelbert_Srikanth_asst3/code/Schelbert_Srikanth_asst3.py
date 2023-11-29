import modern_robotics as mr
import numpy as np
import csv



M01 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]])
M12 = np.matrix([[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]])
M23 = np.matrix([[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]])
M34 = np.matrix([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]])
M45 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]])
M56 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]])
M67 = np.matrix([[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]])
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]
gravity = np.array([0, 0, -9.81])
gravity3 = np.array([0,0,0])
thetalist_0 = np.array([0,0,0,0,0,0])
t = 5
dt = 0.01
damping = 20 # Nms/rad
stiffness = 10 # N/m
spring_pos = np.array([1,1,1,1])
thetamat = []
dthetmat = []

def puppet(thetalist, dthetalist, g, Mlist, Glist, t, dt, damping, stiffness, springPos, restLength, part):
    """A function to create a puppet like UR5 robot."""
    thetamat = []
    dthetmat = []

    if part == 1:
        # Case to be used for part one
        thetamat.append(thetalist)
        for i in range(0,round(t/dt)):
            # for loop to iterate through all of the time steps
            taulist = np.array([0,0,0,0,0,0])
            ftip = np.array([0,0,0,0,0,0])

            ddthetalist = mr.ForwardDynamics(thetalist,dthetalist,taulist,g,ftip,Mlist,Glist,Slist)
            thetalist_next,dthetalist_next = mr.EulerStep(thetalist,dthetalist,ddthetalist,dt)

            # add to the output lists
            thetamat.append(thetalist_next)
            dthetmat.append(dthetalist_next)
            
            thetalist = thetalist_next
            dthetalist = dthetalist_next

        with open('some.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(thetamat)

        return thetamat, dthetmat
    
    elif part == 2:
        # Case to be used for part 2
        thetamat.append(thetalist)
        for i in range(0,round(t/dt)):
            # for loop to iterate through all of the time steps
            taulist = np.array([0,0,0,0,0,0])
            ftip = np.array([0,0,0,0,0,0])

            # Adding damping we now modify the accelerations before calling the euler integration
            ddthetalist = mr.ForwardDynamics(thetalist,dthetalist,taulist,g,ftip,Mlist,Glist,Slist)
            newddthetalist = []
            for l in range(0,len(dthetalist)):
                newddthetalist.append(ddthetalist[l] - (damping * dthetalist[l]))
            thetalist_next,dthetalist_next = mr.EulerStep(thetalist,dthetalist,newddthetalist,dt)

            # add to the output lists
            thetamat.append(thetalist_next)
            dthetmat.append(dthetalist_next)
            
            thetalist = thetalist_next
            dthetalist = dthetalist_next

        with open('some.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(thetamat)

        return thetamat, dthetmat
    
    elif part == 3:
        # case to be used for part 3
        thetamat.append(thetalist)
        for i in range(0,round(t/dt)):
            # for loop to iterate through all of the time steps

            # This code takes in the point and the 
            taulist = np.array([0,0,0,0,0,0])
            ftip = np.array([0,0,0,0,0,0])
            M_3 = M01@M12@M23@M34@M45@M56@M67
            EE_inspace_Trans = mr.FKinSpace(M_3,Slist,thetalist)
            Spring_Pb_frame = mr.TransInv(EE_inspace_Trans)@springPos
            spring_pos_b = np.array([-Spring_Pb_frame[0], -Spring_Pb_frame[1], -Spring_Pb_frame[2]])
            spring_dx = np.linalg.norm(spring_pos_b)

            for s in range(0,len(spring_pos_b)):
                ftip[s+3] = -1 * stiffness * (restLength - spring_dx) * spring_pos_b[s] / spring_dx
                
            # Adding damping we now modify the accelerations before calling the euler integration
            ddthetalist = mr.ForwardDynamics(thetalist,dthetalist,taulist,g,ftip,Mlist,Glist,Slist)
            newddthetalist = []
            for l in range(0,len(dthetalist)):
                newddthetalist.append(ddthetalist[l] - (damping * dthetalist[l]))
            thetalist_next,dthetalist_next = mr.EulerStep(thetalist,dthetalist,newddthetalist,dt)

            # add to the output lists
            thetamat.append(thetalist_next)
            dthetmat.append(dthetalist_next)
            
            thetalist = thetalist_next
            dthetalist = dthetalist_next

        with open('some.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(thetamat)

        return thetamat, dthetmat
            
tmat, dtmat = puppet(thetalist_0,thetalist_0, gravity, Mlist, Glist, t, dt, damping, stiffness, spring_pos, 0, 2)
