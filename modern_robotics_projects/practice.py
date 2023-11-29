import modern_robotics as mr
import numpy as np


# R = np.array([[0, 0, 1],
#               [1, 0, 0],
#               [0, 1, 0]])
# invR = mr.RotInv(R)
# print(invR)

R_sa = np.array([[0,1,0],
                 [0,0,1],
                 [1,0,0]]) # THis is good
print(f"#1: {R_sa}")

R_sb = np.array([[1,0,0],
                 [0,0,1],
                 [0,-1,0]]) 
invR_sb = mr.RotInv(R_sb)
print(f"#2: {invR_sb}")

R_ab = mr.RotInv(R_sa)*R_sb
print(f"#3: {R_ab}")

pb = [1, 2, 3]
ps = R_sb*mr.RotInv(pb)
print(f"#5 {ps}")

w_10 = [1, 2, 0.5]
M_10 = mr.VecToso3(mr.RotInv(w_10))
M_20 = mr.VecToso3(w_10)
print(f"#10: {M_10}")
print(f"#10.2: {M_20}")

w_11 = [[0, 0.5, -1],[-0.5, 0, 2],[1, -2, 0]]
R_11 = mr.MatrixExp3(w_11)
print(f"#11: {R_11}")

R = [[0,0,1],
     [-1,0,0],
     [0,-1,0]]
R_12 = mr.MatrixLog3(R)
print(f"#12: {R_12}")