'''
August 20 2025

The incentive in creating a mesh of the forward kinematics function:
- PROVE that the mesh is actually more simple than we think.
- INTERPOLATE mesh nodes so that we can precompute jacobians and then perform the inverse kinematics.

We will first focus on the mesh for the forward kinematics function.

IF most of the complexity from the visual servoing problem comes from the robot, then we should be able to use the forward kinematics jacobians to solve most simple constraints.

Though, of course, we should be able to generate a mesh for a number of different problems.

We can also generate the mesh for different constraints and see how the mesh changes.
'''
import denavit_hartenberg as dh
import sympy as sp
import numpy as np
import numpy as np
import math
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go
from scipy.spatial import Delaunay

from common_robot_calculations import *


P = dh.DHSympyParams() #this lowkey has to be global, sorry
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
robot = dof3

print("ROBOT:\n",robot.J)

cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
vs.dh_robot.alpha = 1.

kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))
kinova_end = np.deg2rad(np.array([25.336059570312672, 50.57940673828129, -179.4915313720703, -90.7, 30.06742369383573531, -57.420898437500036, 30.88030242919922, 0.5]))

# initialize init Q and de
# initialize init Q and des P

initQ = [1.5] * robot.dof
desQ = [2.0] * robot.dof

desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
print("desired world point:", desP)
print("initial world point:", vs.dh_robot.fkin_eval(*initQ).flatten().tolist())


### HELPER FUNCTIONS ###
def calculate_centroid():
    '''
    calculate the centroid of the simplex
    '''
    pass

def calculate_longest_edge():
    '''
    return the longest edge of the simplex
    '''
    pass

def refine_further_condition():
    '''
    used recursively.
    returns True if the mesh should be refined further.
    returns False if the mesh refinement should halt.

    multiple conditions are possible.
    we will consider Jacobian variation first.
    '''
    pass

def sparse_sample(joint_ranges: list, robot: dh.DenavitHartenbergAnalytic):
    '''
    create a sparse grid over the space 
    create a delaunay mesh of the sparse grid points.
    '''
    
    robot.J()
    

joint_ranges = [(-np.pi/2, np.pi/2)]*robot.dof
