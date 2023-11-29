import modern_robotics as mr
import matplotlib.pyplot as plt
import numpy as np
# from mpl_toolkits.mplot3d import Axes3D

L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095
W1 = 0.109
W2 = 0.082


M = np.array(
    [[-1, 0, 0, L1 + L2], [0, 0, 1, W1 + W2], [0, 1, 0, H1 - H2], [0, 0, 0, 1]]
)

Blist = np.array(
    [
        [0, 1, 0, W1 + W2, 0, L1 + L2],
        [0, 0, 1, H2, -1 * (L1 + L2), 0],
        [0, 0, 1, H2, -1 * L2, 0],
        [0, 0, 1, H2, 0, 0],
        [0, -1, 0, -W2, 0, 0],
        [0, 0, 1, 0, -2, 0],
    ]
).T


tshort = np.loadtxt('short_iterates.csv', delimiter=',')

tlong =np.loadtxt('long_iterates.csv', delimiter=',')



short=[]
long=[]
for theta in tshort:
    temp= mr.FKinBody(M=M, Blist=Blist, thetalist=theta)
    short.append(temp[:3,3])

for theta in tlong:
    temp = mr.FKinBody(M=M, Blist=Blist, thetalist=theta)
    long.append(temp[:3,3])
    
    
# print(short)
# print(long)

fig = plt.figure()
ax = fig.add_subplot(1,1,1, projection='3d')

data2 = np.array(long)
x = data2[:, 0]
y = data2[:, 1]
z = data2[:, 2]
# Create the 3D scatter plot
ax.scatter(x[0], y[0], z[0], c='g', marker='o', label= 'start long')
ax.plot(x, y, z, c='g', linestyle='-', label = 'long')
data = np.array(short)
# Extract x, y, and z coordinates from the data
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]
# Create the 3D scatter plot
ax.scatter(x[0], y[0], z[0], c='r', marker='o', label= 'start short')
ax.plot(x, y, z, c='r', linestyle='-', label='short')
ax.scatter(x[-1], y[-1], z[-1], c='b', marker='x', label='end')

# Set labels for the axes
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.legend()
# Show the plot
plt.show()