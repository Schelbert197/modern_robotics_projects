import modern_robotics as mr
import numpy as np


R = np.array([[0, 0, 1],
              [1, 0, 0],
              [0, 1, 0]])
invR = mr.RotInv(R)
print(invR)

R1 = np.matrix([0,0,1,0,0,0]).T
R2 = np.matrix([0,0,1,0,-1,0]).T
R3 = np.matrix([0,0,1,0.7071,-1.7071,0]).T
# fs = np.matrix([0,0,-1.414,2,0,0]).T
fs = np.matrix([0,0,0,2,0,0]).T

Ri = np.concatenate((R1, R2, R3),axis=1)
ans1 = (Ri.T)@fs
print(ans1)

R12 = np.matrix([0,0,1,1,-1,3]).T
R22 = np.matrix([0,0,1,1,-1,2]).T
R32 = np.matrix([0,0,1,1,-1,1]).T
R42 = np.matrix([0,0,1,1,0,1]).T
fs2 = np.matrix([0,0,10,10,10,0]).T
Ri2 = np.concatenate((R12, R22, R32, R42),axis=1)
ans2 = mr.RotInv(Ri2)@fs2
print(ans2)

S1 = np.matrix([0,0,1,0,0,0]).T
S2 = np.matrix([1,0,0,0,2,0]).T
S3 = np.matrix([0,0,0,0,1,0]).T
Si = np.concatenate((S1, S2, S3),axis=1)
ans = mr.JacobianSpace(Si,[np.pi/2,np.pi/2,1])
print(ans)

B1 = np.matrix([0,1,0,3,0,0]).T
B2 = np.matrix([-1,0,0,0,3,0]).T
B3 = np.matrix([0,0,0,0,0,1]).T
Bi = np.concatenate((B1, B2, B3),axis=1)
anss = mr.JacobianBody(Bi,[np.pi/2,np.pi/2,1])
print(anss)

J1 = np.matrix([-0.105,-0.899,0]).T
J2 = np.matrix([0,0.006,-0.105]).T
J3 = np.matrix([0.006,0,0.889]).T
J4 = np.matrix([-0.045,-0.844,0]).T
J5 = np.matrix([0,0.006,0]).T
J6 = np.matrix([0.006,0,0]).T
J7 = np.matrix([0,0,0]).T
Jb = np.concatenate((J1, J2, J3, J4, J5, J6, J7),axis=1)
A5 = Jb@Jb.T
print(A5)
eig = np.linalg.eig(A5)
print(eig)
ans6 = 1.532**0.5
print(ans6)
L = 1
q = np.pi/2

L1 = np.matrix([1,np.sin(-q)+np.sin(0)+np.sin(0),1+np.cos(-q)+np.cos(0)+np.cos(0)]).T
L2 = np.matrix([1,np.sin(q)+np.sin(0),1+np.cos(-q)+np.cos(0)]).T
L3 = np.matrix([1,L*np.sin(-q),1+np.cos(-q)]).T
L4 = np.matrix([1,0,1]).T
Lb = np.concatenate((L1, L2, L3, L4),axis=1)
f2 = np.matrix([10, 10, 0]).T
ans2 = Lb.T@f2

print(ans2)