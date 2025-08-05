'''
July 28 2025

This program is simply used for testing code.
'''
import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

#import denavit_hartenberg as dh

x1, y1, x2, y2, x3, y3, x4, y4 = sp.symbols("x1 y1 x2 y2 x3 y3 x4 y4")
p1 = sp.Matrix([x1, y1, 1])
p2 = sp.Matrix([x2, y2, 1])
p3 = sp.Matrix([x3, y3, 1])
p4 = sp.Matrix([x4, y4, 1])

p2p_error = (p2-p1)[:2, :] #p1 is end effector, p2 is desired
p2l_error = p1.dot(p2.cross(p3)) # p1 end effector, p2-p3 is desired
l2l_error = p1.dot(p3.cross(p4)) + p2.dot(p3.cross(p4)) #  p1-p2 end effector, p3-p4 desired

print(p2p_error)
print(p2l_error)
print(l2l_error)

variables = x1, y1, x2, y2, x3, y3, x4, y4 
p2peval = (sp.utilities.lambdify(variables[:4], p2p_error, 'numpy'))
p2leval = (sp.utilities.lambdify(variables[:6], p2l_error, 'numpy'))
l2leval = (sp.utilities.lambdify(variables, l2l_error, 'numpy'))

print(p2peval)
print(p2peval(1,2,1,1))
print(p2leval(0,1,0,0,0,0.1))
print(l2leval(3,0,4,3,3,0,3,3))
# P = dh.DHSympyParams() #this lowkey has to be global, sorry
# jntspace, cartspace, taskspace = P.get_params()
# t0, t1, t2, t3, t4, t5, t6, t7, t8, t9 = jntspace
# x, y, z = cartspace
# u, v = taskspace


# dof2_params = [
#                 [t0, 0, 1, 0], 
#                 [t1, 0, 1, 0]
#                 ]

# dylan_dof3_params=[
#                 [ t0, sp.pi/2, 0 , 0 ],
#                 [ t1,  0  ,  0.55, 0 ],
#                 [ t2,  0  ,  0.3, 0 ]
#                 ]


# kinova_dof7_params = [
#     [t0,      sp.pi,   0.0,   0.0],
#     [t1,      sp.pi/2, 0.0, -(0.1564 + 0.1284)],
#     [t2 +sp.pi, sp.pi/2, 0.0, -(0.0054 + 0.0064)],
#     [t3 +sp.pi, sp.pi/2, 0.0, -(0.2104 + 0.2104)],
#     [t4 +sp.pi, sp.pi/2, 0.0, -(0.0064 + 0.0064)],
#     [t5 +sp.pi, sp.pi/2, 0.0, -(0.2084 + 0.1059)],
#     [t6 +sp.pi, sp.pi/2, 0.0, 0.0],
#     [t7 +sp.pi,    sp.pi, 0.0, -(0.1059 + 0.0615)],
# ]

# dof2 = dh.DenavitHartenbergAnalytic(dof2_params, P)
# dof3 = dh.DenavitHartenbergAnalytic(dylan_dof3_params, P)
# kinova = dh.DenavitHartenbergAnalytic(kinova_dof7_params, P)
# robot = kinova

# cam1 = dh.Camera(0,0,0,[0,0,5], 5,5, 0, 0) #looks down directly at the scene, basically replicates the scene
# cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,5], 5,5,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
# cameras=[cam2]

# test = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)

# print("\ntest.F")
# print(test.F)
# print("\ntest.J")
# print(test.J)

# test.calc_lipschitz()

