import modern_robotics as mr
import numpy as np
import csv
import matplotlib.pyplot as plt

lin_err = []
ang_err = []


##### CREATED FUNCTION #####
def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
        """Computes inverse kinematics in the body frame for an open chain robot

        :param Blist: The joint screw axes in the end-effector frame when the
                    manipulator is at the home position, in the format of a
                    matrix with axes as the columns
        :param M: The home configuration of the end-effector
        :param T: The desired end-effector configuration Tsd
        :param thetalist0: An initial guess of joint angles that are close to
                        satisfying Tsd
        :param eomg: A small positive tolerance on the end-effector orientation
                    error. The returned joint angles must give an end-effector
                    orientation error less than eomg
        :param ev: A small positive tolerance on the end-effector linear position
                error. The returned joint angles must give an end-effector
                position error less than ev
        :return thetalist: Joint angles that achieve T within the specified
                        tolerances,
        :return success: A logical value where TRUE means that the function found
                        a solution and FALSE means that it ran through the set
                        number of maximum iterations without finding a solution
                        within the tolerances eomg and ev.
        Uses an iterative Newton-Raphson root-finding method.
        The maximum number of iterations before the algorithm is terminated has
        been hardcoded in as a variable called maxiterations. It is set to 20 at
        the start of the function, but can be changed if needed.

        Example Input:
            Blist = np.array([[0, 0, -1, 2, 0,   0],
                            [0, 0,  0, 0, 1,   0],
                            [0, 0,  1, 0, 0, 0.1]]).T
            M = np.array([[-1, 0,  0, 0],
                        [ 0, 1,  0, 6],
                        [ 0, 0, -1, 2],
                        [ 0, 0,  0, 1]])
            T = np.array([[0, 1,  0,     -5],
                        [1, 0,  0,      4],
                        [0, 0, -1, 1.6858],
                        [0, 0,  0,      1]])
            thetalist0 = np.array([1.5, 2.5, 3])
            eomg = 0.01
            ev = 0.001
        Output:
            (np.array([1.57073819, 2.999667, 3.14153913]), True)
        """
        thetalist = np.array(thetalist0).copy()
        i = 0
        maxiterations = 20 # maximum the function may iterate before it says unsuccessful
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
            or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
        
        csv_stuff = [] # dummy variable for passing things to be created in the csv file
        while err and i < maxiterations:
            thetalist = thetalist \
                        + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, \
                                                            thetalist)), Vb)
            i = i + 1
            Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))
            err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
                or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
            
            thetalist1 = thetalist
            se3 = mr.FKinBody(M, Blist, thetalist)
            # Create a loop that will triage the thetas and put them in the -2pi to 2pi range.
            for k in range(len(thetalist1)):
                if thetalist1[k] > 2*np.pi:
                    temp = thetalist1[k] % 2*np.pi
                    thetalist1[k] = temp
                elif thetalist1[k] < -2*np.pi:
                    temp = thetalist1[k] % 2*np.pi
                    thetalist1[k] = temp - 2*np.pi
            
            # Create the list of things that will be output to csv
            csv_stuff.append(thetalist)

            # Create the terminal printouts
            print(f"Iteration number: {i}")
            print(f"Joint Vector: {thetalist1}")
            print(f"SE(3) Config:\n {se3}")
            print(f"Error Twist Vb: {Vb}")
            print(f"Angular Error: {np.linalg.norm([Vb[0], Vb[1], Vb[2]])}")
            lin_err.append(np.linalg.norm([Vb[0], Vb[1], Vb[2]])) # this was printed out for plotting purposes and can be ignored
            print(f"Linear Error: {np.linalg.norm([Vb[3], Vb[4], Vb[5]])}")
            ang_err.append(np.linalg.norm([Vb[3], Vb[4], Vb[5]])) # his was printed out for plotting purposes and can be ignored

        # Function to send stuff to csv
        with open('some.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(csv_stuff)

        # Return the last value and whether an error occured or not (aka whether it converged or didn't finish)
        return (thetalist, not err)

##### END OF FUNCTION, REST CAN BE IGNORED BY GRADER #####

L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095
W1 = 0.109
W2 = 0.082

Tsd = np.array([[0.7071, 0, 0.7071, -0.3],
                [0.7071, 0, -0.7071, -0.5],
                [0, 1, 0, 0.5],
                [0, 0, 0, 1]])
M = np.array([[-1, 0, 0, L1+L2],
                [0, 0, 1, W1+W2],
                [0, 1, 0, H1-H2],
                [0, 0, 0, 1]])
B_list = np.array([[0, 1, 0, W1+W2, 0, L1+L2],
                [0, 0, 1, H2, -L1-L2, 0],
                [0, 0, 1, H2, -L2, 0],
                [0, 0, 1, H2, 0, 0],
                [0, -1, 0, -W2, 0, 0],
                [0, 0, 1, 0, 0, 0]]).T

thetalist0 = np.array([0,0,0.7854,0,0.7854,0.7854])
# thetalist0 = np.array([03.8394,5.1428,1.5628,3.4872,0.0533,2.3734])

eomg = 0.001
ev = 0.0001

ans2 = IKinBodyIterates(B_list, M, Tsd, thetalist0, eomg, ev)
print(ans2)

long_l_err = [0.8992644829429922, 0.36881044083888553, 2.809982406866833, 2.920278553345487, 0.8924889256697659, 2.412692546318661, 2.3270831079253305, 1.701975545115132, 0.745312631410777, 0.1266738147216591, 0.2115566781559602, 0.10187540152064448, 0.008451693625004619, 0.00011976315517662045]
long_a_err = [1.0801103983621017, 0.9671905799313381, 2.1156589341710648, 1.9309378351331747, 0.7160723823838983, 1.027625667921509, 0.870718110788954, 0.618992493626164, 0.398596933740028, 0.23624169652937485, 0.11148386337526642, 0.028928577314525097, 0.0007909253769445142, 3.3787582419949935e-05]

short_l_err = [0.039737419089052414, 0.012784557018631314, 0.0012792464410914643, 9.721197343117593e-06]
short_a_err = [0.06423989490344827, 0.013139208872938079, 0.0010778666627332787, 7.296202930462472e-06]

fig = plt.figure()

ax = fig.add_subplot()


ax.plot(short_l_err, linestyle='-', c='r', label='short linear error')
ax.plot(long_l_err, linestyle='-', c='g', label='long linear error')
plt.legend()

fig1 = plt.figure()
dx = fig1.add_subplot()
dx.plot(long_a_err, linestyle='-', c='r', label='long angular error')
dx.plot(short_a_err, linestyle='-', c='g', label='short angular error')

# Set labels for the axes
ax.set_xlabel('Iteration')
ax.set_ylabel('Liner error')
dx.set_xlabel('Iteration')
dx.set_ylabel('Angular error')
plt.legend()
# Show the plot

plt.show()