import modern_robotics as mr
import numpy as np


R = np.array([[0, 0, 1],
              [1, 0, 0],
              [0, 1, 0]])
invR = mr.RotInv(R)

R_13 = np.array([[-0.7071, 0, -0.7071],
                 [0, 1, 0],
                 [0.7071, 0, -0.7071]])

R_s2 = np.array([[-0.6964, 0.1736, 0.6964],
                 [-0.1228, -0.9848, 0.1228],
                 [0.7071, 0, 0.7071]])

R_25 = np.array([[-0.7566, -0.1198, -0.6428],
                 [-0.1564, 0.9877, 0],
                 [0.6348, 0.1005, -0.7661]])

R_12 = np.array([[0.7071, 0, -0.7071],
                 [0, 1, 0],
                 [0.7071, 0, 0.7071]])

R_34 = np.array([[0.6428, 0, -0.7660],
                [0, 1, 0],
                [0.7660, 0, 0.6428]])

R_s6 = np.array([[0.9418, 0.3249, -0.0859],
                 [0.3249, -0.9456, -0.0151],
                 [-0.0861, -0.0136, -0.9962]])

R_6b = np.array([[-1, 0, 0],
                 [0, 0, 1],
                 [0, 1, 0]])

R_s1 = R_s2@mr.RotInv(R_12)

R_23 = mr.RotInv(R_12)@R_13

R_45 = mr.RotInv(R_34)@mr.RotInv(R_23)@R_25

R_56 = mr.RotInv(R_25)@mr.RotInv(R_s2)@R_s6

R_sb = R_s6@R_6b


omega1, theta1 = mr.AxisAng3(mr.so3ToVec(mr.MatrixLog3(R_s1)))
omega2, theta2 = mr.AxisAng3(mr.so3ToVec(mr.MatrixLog3(R_12)))
omega3, theta3 = mr.AxisAng3(mr.so3ToVec(mr.MatrixLog3(R_23)))
omega4, theta4 = mr.AxisAng3(mr.so3ToVec(mr.MatrixLog3(R_34)))
omega5, theta5 = mr.AxisAng3(mr.so3ToVec(mr.MatrixLog3(R_45)))
omega6, theta6 = mr.AxisAng3(mr.so3ToVec(mr.MatrixLog3(R_56)))

# Omegas
print(omega1)
print(omega2)
print(omega3)
print(omega4)
print(omega5)
print(omega6)

# Thetas (they are multiplied by -1 if omega is flipped)
print(-theta1, -theta2, -theta3, -theta4, theta5, theta6)

print(R_sb)
