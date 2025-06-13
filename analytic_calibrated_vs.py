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
u,v,w = sp.symbols('u:w', real=True)

# l1 and l2 
l1,l2,l3 = sp.symbols('l(1:4)', positive=True)

#CAMERA 
fx = sp.Symbol('fx', real=True)
fy = sp.Symbol('fy', real=True)
cx = sp.Symbol('cx', real=True)
cy = sp.Symbol('cy', real=True)

rotation_symbols= np.array(sp.symbols('r1(1:4), r2(1:4), r3(1:4)'))
R = sp.Matrix([rotation_symbols[0:3], rotation_symbols[3:6], rotation_symbols[6:9]]) #over parameterized 
t = sp.Matrix(sp.symbols('t1(1:4)'))

E = R.col_insert(3, t).row_insert(3, sp.Matrix([[0,0,0,1]])) #extrinsic matrix

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
    '''
    Returns the general denavit hartenberg transformation matrix for one link, i.
    theta_i is the angle about the z(i-1) axis between x(i-1) and x(i).
    alpha_i is the angle about the x(i) axis between z(i-1) and z(i).
    r_i is the distance between the origin of frame i-1 and i along the x(i) direction.
    d_i is the distance from x(i-1) to x(i) along the z(i-1) direction.
    '''
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
   
    Returns an array of positions for plotting, and the final DH transform matrix of the EE.
    '''
    l1 = 0.55
    l2 = 0.30
    T01= transformation_matrix_DH(t0, sp.pi/2, 0, 0)
    T12= transformation_matrix_DH(t1, 0, l1, 0)
    T23= transformation_matrix_DH(t2, 0, l2, 0)
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

    #print(positions)

    return sp.Matrix(positions), EE
    

def wireframe(joints):
    '''
    This function serves to plot the wireframe of our robot points.
    Input is a series of points, which will be connected in the plot sequentially.
    Fixed axis limits prevent automatic caling perspective.
    '''
    print(joints)
    x = joints[:,0]
    y = joints[:,1]
    z = joints[:,2]

    fig=plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(x,y,z, marker='o', linestyle='-')

    xlim=[-1,1]
    ylim=[-1,1]
    zlim=[-1,1]
    # Set fixed axis limits
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)

    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('3D Line Plot')
    plt.show()

def compute_analytic_jac(Q):
    '''
    Analytic Jacobian of image.

    Q : joint vector
    P : the camera projection matrix :-))

    x=P*X

    And then get the Jacobian of x, as dx
    '''
    x = P * X
    dx = x.jacobian([u,v,w])
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

def rtb_model():
    '''
    Just to verify everything is okay, I have the rtb DH robot initialized here.
    '''

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
    robot.plot([0,0,sp.pi/2], block=True) #'''


def main():
    joints, EE =denavit_hartenburg_dylan_3dof(u,v,w)
    print("EE:\n",EE)
    position = ((EE[:,3]).T)
    print(position)
    u_des,v_des,w_des = [0,sp.pi/2,sp.pi/2]
    reps_des=[(u,u_des), (v,v_des), (w, w_des)]
    #wireframe(joints.subs(reps_des))
    #print(position.subs(reps_des))

    #position is our analytical cartesian world point!


main()
'''
Now we have the end effector of the robot and we also have the camera. 
If we only care about translation and not rotation, 
we can simply multiply our EE world point by our camera and get the projected point.

With this 
'''