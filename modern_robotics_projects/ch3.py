import modern_robotics as mr
import numpy as np


# R = np.array([[0, 0, 1],
#               [1, 0, 0],
#               [0, 1, 0]])
# invR = mr.RotInv(R)
# print(invR)

T_sb = np.array([[1,0,0,0],[0,0,1,2],[0,-1,0,0],[0,0,0,1]]) # THis is good
# T_sb_inv = mr.RotInv(T_sb)
T_sb_inv = np.linalg.inv(T_sb)

print(T_sb_inv)

T_sa = np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,1],[0,0,0,1]])
T_sa_inv = mr.RotInv(T_sa)
T_ab = T_sa_inv@T_sb
print(T_ab)

V_s = [3,2,1,-1,-2,-3]
V_a = mr.Adjoint(np.linalg.inv(T_sa))@V_s
print(f"Va: {V_a}")

S_8 = mr.MatrixLog6(T_sa)
S_8_ans = mr.so3ToVec(S_8)
print(f"S8: {S_8_ans}")

S_9 = [0,1,2,3,0,0]
Se_s9 = mr.VecTose3(S_9)
S_9_ans = mr.MatrixExp6(Se_s9)
print(S_9_ans)

F_b = [1,0,0,2,1,0]
F_s = mr.Adjoint(T_sb_inv).T@F_b
print(f"Fs: {F_s}")

T_11 = np.array([[0,-1,0,3],[1,0,0,0],[0,0,1,1],[0,0,0,1]])
T_11_ti = mr.TransInv(T_11)
print(T_11_ti)

V_12 = [1, 0, 0, 0, 2, 3]
V_12_a = mr.VecTose3(V_12)
print(V_12_a)

s = [1, 0, 0]
q = [0, 0, 2]
h = 1
S = mr.ScrewToAxis(q, s, h)
print(S)

S_theta = np.array([[0,-1.5708,0,2.3562],[1.5708,0,0,-2.3562],[0,0,0,1],[0,0,0,0]])
T_14 = mr.MatrixExp6(S_theta)
print(T_14)

T_15 = np.array([[0,-1,0,3],[1,0,0,0],[0,0,1,1],[0,0,0,1]])
T_15_l = mr.MatrixLog6(T_15)
print(T_15_l)