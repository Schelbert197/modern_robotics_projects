import modern_robotics as mr
import numpy as np
import math

L = 1
S1 = np.matrix([0,0,1,0,-L,0]).T
S2 = np.matrix([0,1,0,0,0,L]).T
S3 = np.matrix([0,1,0,L,0,L*(math.sqrt(3)+1)]).T
S4 = np.matrix([0,1,0,(1-math.sqrt(3))*L,0,(2+math.sqrt(3))*L]).T
S5 = np.matrix([0,0,0,0,0,1]).T
S6 = np.matrix([0,0,1,0,-L*(2+math.sqrt(3)),0]).T

Ree = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])
pee = np.matrix([L*(2+math.sqrt(3)),0,L*(1+math.sqrt(3))]).T
M = mr.RpToTrans(Ree, pee)
print(M)

Si = np.concatenate((S1, S2, S3, S4, S5, S6),axis=1)
print(Si)

B1 = S2 = np.matrix([0,0,1,0,L*(1+math.sqrt(3)),0]).T
B2 = np.matrix([0,1,0,L*(1+math.sqrt(3)),0,-L*(1+math.sqrt(3))]).T
B3 = np.matrix([0,1,0,L*(2+math.sqrt(3)),0,-L]).T
B4 = np.matrix([0,1,0,2*L,0,0]).T
B5 = np.matrix([0,0,0,0,0,1]).T
B6 = np.matrix([0,0,1,0,0,0]).T

Bi = np.concatenate((B1,B2,B3,B4,B5,B6),axis=1)
print(Bi)

theta_list = [-np.pi/2,np.pi/2,np.pi/3,-np.pi/4,1,np.pi/6]
T_4 = mr.FKinSpace(M,Si,theta_list)
print(T_4)

T_5 = mr.FKinBody(M,Bi,theta_list)
print(T_5)