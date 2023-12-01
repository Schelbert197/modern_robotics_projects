import modern_robotics as mr
import numpy as np
import sympy as sym

Xs = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

Xend = np.array([[0, 0, 1, 1],
                [1, 0, 0, 2],
                [0, 1, 0, 3],
                [0, 0, 0, 1]])

ans5 = mr.QuinticTimeScaling(5,3)
print(f"Ans 5:{ans5}")

ans6 = mr.ScrewTrajectory(Xs,Xend,10,10,3)

print(f"Ans 6:{ans6}")


ans7 = mr.CartesianTrajectory(Xs,Xend,10,10,5)



print(f"Ans 6:{ans7}")
a3,a4,a5,T = sym.symbols(r'a_3,a_4,a_5,T')
lhs3 = sym.Matrix([3*a3*T**2 + 4*a4*T**3 + 5*a5*T**4,a3*T**3 + a4*T**4 + a5*T**5,6*a3*T + 12*a4*T**2 + 20*a5*T**3])
# define right hand side as a Matrix
rhs3 = sym.Matrix([0,1,0])

eq3p = sym.Eq(lhs3, rhs3)
soln = sym.solve(eq3p,[a3,a4,a5], dict=True)
print(soln)