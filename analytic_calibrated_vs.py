'''
June 11 

Analytic visual servoing. Image based, uncalibrated (in practice). Let's see how the spectral radius changes.
'''
import numpy as np
import math
from spatialmath import SE3
from spatialmath.base import e2h, h2e
import machinevisiontoolbox as mvtb
from machinevisiontoolbox import CentralCamera
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go

import sympy as sp
import successive_mesh_refinement as smr
import common_robot_calculations as crc

# for a desired position, what is the analytic Newton's equation?

# u and v <==> theta1 and theta2
u = sp.Symbol('u', real=True)
v = sp.Symbol('v', real=True)
# l1 and l2 
l1 = sp.Symbol('l1', positive=True)
l2 = sp.Symbol('l2', positive=True)
l1=1;l2=1; #for ease let's just have lengths be 1

#declare the forward kin functions
f1 = l1 * sp.cos(u) + l2 * sp.cos(u+v)  # x coord
f2 = l1 * sp.sin(u) + l2 * sp.sin(u+v)  # y coord
f3 = 0 #z coord, robot lies on x-y plane at origin

#CAMERA 
fx = sp.Symbol('fx', real=True)
fy = sp.Symbol('fy', real=True)
cx = sp.Symbol('cx', real=True)
cy = sp.Symbol('cy', real=True)

rotation_symbols= np.array(sp.symbols('r1(1:4), r2(1:4), r3(1:4)'))
R = sp.Matrix([rotation_symbols[0:3], rotation_symbols[3:6], rotation_symbols[6:9]]) #over parameterized 
t = sp.Matrix(sp.symbols('t1(1:4)'))

E = R.col_insert(3, t) #extrinsic matrix
#TODO: Add the homog row at bottom


print(R)
print(t)

print(E)

K = sp.Matrix([[fx, 0, cx],[0, fy, cy], [0,0,1]]) #intrinsic matrix

print(K)

P = K * E

print(P)

X = sp.Matrix([[f1],[f2],[f3],[1]])

x = P * X

print(x[0])
print(x[1])
print(x[2])

def transformation_matrix_DH(theta_i, alpha_i, r_i, d_i):
    alpha, r, theta, d = sp.symbols('alpha, r, theta, d', real=True)
    general_DH = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), r*sp.cos(theta)],
                            [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), r*sp.sin(theta)],
                            [0, sp.sin(alpha), sp.cos(alpha), d],
                            [0,0,0,1]])
    #print(general_DH)
    DH = general_DH.subs([(alpha, alpha_i), (r, r_i), (theta, theta_i), (d, d_i)])
    print(DH)
    return DH

def denavit_hartenburg_dylan_3dof(t0, t1, t2):
    '''
    "We simulate a 3DOF arm, where the first joint rotates about the z-axis, and
    the other two about the y-axis. The link lengths were set to l1 = 0.55 and l2 = 0.30. 
    
    The denavit hartenburg matrix is:
    theta   alpha   r   d
    [[ t0, pi/2,  0   , 0 ],
     [ t1,  0  ,  0.55, 0 ],
     [ t2,  0  ,  0.30, 0 ]]
   
    '''
    T01= transformation_matrix_DH(t0, sp.pi/2, 0, 0)
    T12= transformation_matrix_DH(t1, 0, 2, 0)
    T23= transformation_matrix_DH(t2, 0, 1, 0)
    transforms = [T01, T12, T23]

    EE = T01 * T12 * T23
    print("EE:" ,EE)

    positions = []  # Base at origin
    T_current = sp.eye(4)

    for T in transforms:
        T_current = T_current * T
        pos = np.transpose(T_current[:,3])[0][:3]
        print(pos)
        positions.append(pos)

    print(positions)

    return np.array(positions) 
    

def wireframe(joints):
    print(joints)
    x = joints[:,0]
    y = joints[:,1]
    z = joints[:,2]

    fig=plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(x,y,z, marker='o', linestyle='-')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('3D Line Plot')
    plt.show()

joints=denavit_hartenburg_dylan_3dof(0,0,sp.pi/2)
wireframe(joints)

    


def compute_analytic_jac(Q):
    '''
    Analytic Jacobian of image.

    Q : joint vector
    P : the camera projection matrix :-))

    x=P*X

    And then get the Jacobian of x, as dx
    '''
    x = P * X
    dx = x.jacobian([u,v])
    dx=dx.subs([(u, Q[0]), (v, Q[1])])
    print(dx)
    print(sp.shape(dx))
    return dx

def spectral_radius(u_,v_, udes,vdes):
    '''
    spectral radius
    '''
    J = compute_analytic_jac([u_, v_])
    try:
        B = J.inv()
    except:
        print("error")
        B= None

    dF = compute_analytic_jac([udes, vdes])

    G = B*dF
    print("G:")
    print(G)

    

'''
Goal:

Find the analytic equations for visual servoing ! Find the spectral radius too. 
We have a ton of hyperparameters, but I only care about u and v.

[u,v] = [u,v] - B*F*alpha, where F is the residual in image position, and B is the central differences Jacobian from the beginning... 
We cannot access the true Jacobian in uncalibrated visual servoing. Should we still... TRY to get it?
'''
 
#spectral_radius(0,1.5,.5,1.3)

'''
Start with best condition: two orth-to-scene cameras (fixed or moving? probably start with two fixed ) 


Base coord in world frame is  0,0,0. Last col in extrinsic will tell you the position of the camera, 

How to break down the fkin into x,y,x? 
- use DH: convention to specify a joint wrt to previous joint 
- plug params into formula to get homog matrix 
- start multiplying the homog matrices together to get the end effector - [ R | t ]

Robotics textbook:
- Modern Robotics (twists, exp coords)
- Craig from 464 (has DH params)
'''

#Just to verify everything is okay, I have the rtb DH robot initialized here.

from roboticstoolbox import DHRobot, RevoluteDH
from math import pi

# Define the 3 links using standard DH parameters
link1 = RevoluteDH(alpha=pi/2, a=0, d=0)
link2 = RevoluteDH(alpha=0,    a=2, d=0)
link3 = RevoluteDH(alpha=0,    a=1, d=0)

# Create the robot
robot = DHRobot([link1, link2, link3], name='3DOF_Robot')
print(robot.dhunique())
# Print the robot description
print(robot)
robot.plot([0,0,sp.pi/2], block=True)#'''
