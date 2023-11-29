import matplotlib.pyplot as plt
import modern_robotics as mr
import numpy as np
import csv
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

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

# thetalist0 = np.array([0,0,0.7854,0,0.7854,0.7854])
# thetalist0 = np.array([03.8394,5.1428,1.5628,3.4872,0.0533,2.3734])

eomg = 0.001
ev = 0.0001

short_list = np.loadtxt('short_iterates.csv', delimiter=',')
long_list = np.loadtxt('long_iterates.csv', delimiter=',')

xlist = []
ylist = []
zlist = []

xlistl = []
ylistl = []
zlistl = []

for i in range(len(short_list)):
    q = mr.FKinBody(M, B_list, short_list[i])
    xlist.append(q[0][3])
    ylist.append(q[1][3])
    zlist.append(q[2][3])


for k in range(len(long_list)):
    p = mr.FKinBody(M, B_list, long_list[k])
    xlistl.append(p[0][3])
    ylistl.append(p[1][3])
    zlistl.append(p[2][3])

fig = plt.figure()
ax = fig.add_subplot(projection='3d')


# Create the 3D scatter plot
ax.scatter(xlist[0], ylist[0], zlist[0], c='g', marker='o', label= 'start short')
ax.plot(xlist, ylist, zlist, c='g', linestyle='-', label = 'short')
ax.scatter(xlist[-1], ylist[-1], zlist[-1], c='b', marker='x', label='end short')

# Create the 3D scatter plot
ax.scatter(xlistl[0], ylistl[0], zlistl[0], c='r', marker='o', label= 'start long')
ax.plot(xlistl, ylistl, zlistl, c='r', linestyle='-', label='long')
ax.scatter(xlistl[-1], ylistl[-1], zlistl[-1], c='y', marker='x', label='end long')

# Set labels for the axes
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.legend()
# Show the plot

plt.show()