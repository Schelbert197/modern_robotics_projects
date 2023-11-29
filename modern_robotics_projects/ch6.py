import modern_robotics as mr
import numpy as np

x = 1
y = 1
R = np.array([[2*x, 0],
              [0, 2*y]])
# invR = mr.RotInv(R)
# print(invR)
R_inv = np.linalg.inv(R)
print(R_inv)

ans = 2.25*0.2
print(ans)

Tsd = np.array([[-0.585, -0.811, 0, 0.076],
              [0.811, -0.585, 0, 2.608],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

M = np.array([[1, 0, 0, 3],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

S_list = np.array([[0, 0, 1, 0, 0, 0],
                    [0, 0, 1, 0, -1, 0],
                    [0, 0, 1, 0, -2, 0]]).T

thetalist0 = np.array([0.7854,0.7854,0.7854])
eomg = 0.001
ev = 0.0001

ans2 = mr.IKinSpace(S_list, M, Tsd, thetalist0, eomg, ev)
print(ans2)