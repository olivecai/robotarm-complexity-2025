'''
July 23 2025

This program uses Kantorovich principles in an algorithm designed to:
- use as few jacobians as possible to complete the inverse kinematics
- move out of singular positions

TODO: need to implement constrained nonlinear system as unconstrained with penalty.

In contrast to the heavy theory present in Kantorovich's theorem, we will be using very empirical 'quick fixes' in this program, 
just to visualize what might work or not.

So, what are 'quick fixes' that we think could work based on our 2,3, kinova-dof?

- h must be less than 4 or so, maybe 5... 
- closer we are == dampening closer to 1 (farther, dampening closer to 0)
- instead of using the global lipschitz constant, use the spectral radius at that points (possible multiply by a factor a little greater than 1)
- b must be less than 1 or 1.5 ish

And for all of the above, let them be guidance, or suggestions, but provide proper fallback if the method begins to fail.]
This way we don't need to be as worried about failure.

if B is very large, then we are at a singularity and should move out of that configuration.



This program partially exists as a last ditch effort to not give up on the Kantorovich semilocal convergence...

'''

import denavit_hartenberg as dh
import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

import denavit_hartenberg as dh

####################################
##### SET UP PARAMS AND ROBOT ######
#######################################

P = dh.DHSympyParams()
jntspace, cartspace, taskspace = P.get_params()
t0, t1, t2, t3, t4, t5, t6, t7, t8, t9 = jntspace
x, y, z = cartspace
u, v = taskspace

dof2_params = [
                [t0, 0, 1, 0], 
                [t1, 0, 1, 0]
                ]

dylan_dof3_params=[
                [ t0, sp.pi/2, 0 , 0 ],
                [ t1,  0  ,  0.55, 0 ],
                [ t2,  0  ,  0.3, 0 ]
                ]


kinova_dof7_params = [
    [t0,      sp.pi,   0.0,   0.0],
    [t1,      sp.pi/2, 0.0, -(0.1564 + 0.1284)],
    [t2 +sp.pi, sp.pi/2, 0.0, -(0.0054 + 0.0064)],
    [t3 +sp.pi, sp.pi/2, 0.0, -(0.2104 + 0.2104)],
    [t4 +sp.pi, sp.pi/2, 0.0, -(0.0064 + 0.0064)],
    [t5 +sp.pi, sp.pi/2, 0.0, -(0.2084 + 0.1059)],
    [t6 +sp.pi, sp.pi/2, 0.0, 0.0],
    [t7 +sp.pi,    sp.pi, 0.0, -(0.1059 + 0.0615)],
]

dof2 = dh.DenavitHartenbergAnalytic(dof2_params, P)
dof3 = dh.DenavitHartenbergAnalytic(dylan_dof3_params, P)
kinova = dh.DenavitHartenbergAnalytic(kinova_dof7_params, P)
robot = dof2

print(robot.J)

#############################################333

'''
'''