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

class camera:
    '''
    creates a camera with actual parameters
    '''
    def __init__(self, rot_xaxis, rot_yaxis, rot_zaxis, translation, fx, fy, cx, cy):
        '''
        rot axis is a radian rotation around a specified axis.
        '''
        self.K=sp.Matrix([[fx, 0, cx],[0, fy, cy], [0,0,1]]) #intrinsic matrix

        rx = sp.Matrix([[1,0,0],[0,sp.cos(rot_xaxis), -sp.sin(rot_xaxis)],[0,sp.sin(rot_xaxis), sp.cos(rot_xaxis)]])
        ry= sp.Matrix([[sp.cos(rot_yaxis), 0, sp.sin(rot_yaxis)],[0,1,0],[-sp.sin(rot_yaxis), 0, sp.cos(rot_yaxis)]])
        rz = sp.Matrix([[sp.cos(rot_zaxis), -sp.sin(rot_zaxis), 0], [sp.sin(rot_zaxis), sp.cos(rot_zaxis),0],[0,0,1]])
        
        R = rx*ry*rz
        t=sp.Matrix([translation[0],translation[1],translation[2]])
        
        E = R.col_insert(3,t)

        self.E = E

        self.P = self.K*self.E

    def projectpoint(self, worldpoint):
        x = self.P * worldpoint
        #print("projection point before flatten:")
        #print(x)
        x[0]=x[0]/x[2]
        x[1]=x[1]/x[2]
        x[2]=1#'''
        return sp.Matrix([[x[0]],[x[1]]])



#focal length and center doesnt affect the convergence, which makes sense since it just scaling...
fx=10; fy=10;
cx=500; cy=500;

rotation_symbols= np.array(sp.symbols('r1(1:4), r2(1:4), r3(1:4)'))
R = sp.Matrix([rotation_symbols[0:3], rotation_symbols[3:6], rotation_symbols[6:9]]) #over parameterized 
r11=1;r12=0; r13=0
r21=0; r22=1;r23=0
r31=0; r32=0; r33=1
R=sp.Matrix([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])
t = sp.Matrix(sp.symbols('t1(1:4)'))
t= sp.Matrix([[0],[0],[0]]) #for now

E = R.col_insert(3, t) #extrinsic matrix

print("SETUP:")
print("extrinsic R|t:", E)

K = sp.Matrix([[fx, 0, cx],[0, fy, cy], [0,0,1]]) #intrinsic matrix
print("intrinsic:", K)

P = K * E

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
    #print(DH)
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
    l1 = 55/100
    l2 = 30/100
    T01= transformation_matrix_DH(t0, sp.pi/2, 0, 0)
    T12= transformation_matrix_DH(t1, 0, l1, 0)
    T23= transformation_matrix_DH(t2, 0, l2, 0)
    transforms = [T01, T12, T23]

    EE = T01 * T12 * T23

    positions = []  # Base at origin
    T_current = sp.eye(4)

    for T in transforms:
        T_current = T_current * T
        pos = np.transpose(T_current[:,3])[0][:3]
        positions.append(pos)

    #print(positions)

    return sp.Matrix(positions), EE

def denavit_hartenburg_planar_2dof(t0, t1, t2):
    '''
    Your basic 2DOF planar robot. 
    
    The denavit hartenburg matrix is:
    theta   alpha   r   d
     [ t0,  0  ,  0.55, 0 ],
     [ t1,  0  ,  0.30, 0 ]]
   
    Returns an array of positions for plotting, and the final DH transform matrix of the EE.
    '''
    l0 = 0.55
    l1 = 0.30
    T01= transformation_matrix_DH(t1, 0, l1, 0)
    T12= transformation_matrix_DH(t2, 0, l2, 0)
    transforms = [T01, T12]

    EE = T01 * T12 
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
    Fixed axis limits will prevent automatic scaling perspective.
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

def compute_analytic_jac(cameras):
    '''
    Analytic Jacobian of the projected image.

    Specifically for 3DOF.

    Q : joint vector
    P : the camera projection matrix :-))

    x=P*X

    And then get the Jacobian of x, as dx
    '''
    joints, EE =denavit_hartenburg_dylan_3dof(u,v,w)
    world_position = ((EE[:,3]))
    x = cameras[0].projectpoint(world_position)
    print("x:", x)
    for i in range(1, len(cameras)):
        x = x.col_join(cameras[i].projectpoint(world_position))
    dx = x.jacobian([u,v,w]) #get the jacobian of the projected point...

    print("\nAnalytic Jacobian:\n")
    print(dx)
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

def evals_and_sr(A):
    
    # Get the eigenvalues
    try:
        eigenvals = A.eigenvals()  # returns dict: {eigenvalue: multiplicity}
        # Compute the spectral radius (max absolute eigenvalue)
        return list(eigenvals.keys()), sp.Max(*[sp.Abs(lam) for lam in eigenvals.keys()])
    except:
        return None, None
    
def plot(desired_joint_position, cameras, resolution, jointlimits):
    #for better generality if we move to higher dof, pass each initial joint configuration through forward kinematics to plot loss
    joint_ranges = [np.linspace(low, high, resolution) for (low, high) in jointlimits]
    grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)

    # preallocate error arrays (same shape as grid, but without dof)
    permuts_over_linspaces_shape = Q_grid.shape[:-1]
    permuts_over_linspaces = np.zeros(permuts_over_linspaces_shape)

    i=0;

    u_des,v_des,w_des = desired_joint_position
    #u_des, v_des, w_des = sp.symbols('u_des, v_des, w_des', real=True)
    reps_des=[(u,u_des), (v,v_des), (w, w_des)]

    analytic_jac = compute_analytic_jac(cameras)
    dF = analytic_jac.subs(reps_des)
    for idx in np.ndindex(permuts_over_linspaces_shape): #iter over every permut over linspace of q0...qn 
        '''
        for every iteration/permutation of joint space configuration,
        compute the spectral radius with the camera(s) for the desired joint configuration.
        to save computation time, cache the analytic jacobian and the constant dF.
        '''
        Q = Q_grid[idx].copy()
        u0,v0,w0 = Q
        reps_0=[(u,u0), (v,v0), (w, w0)]
        J = analytic_jac.subs(reps_0)
        try:
            B=J.pinv()
        except:
            B=None

        if B:
            I = sp.eye(3)
            A = (I - B*dF)
            evals, sr = evals_and_sr(A)
            print("\nsr:")
            print(sr)
        else:
            evals, sr = None, None
        
        '''
        Right now this cartesian conversion is in totally the wrong place
        # turn u0 v0 w0 into x y z (aka joint --> cartesian)
        _, EE = denavit_hartenburg_dylan_3dof(u0, v0, w0)
        world_init = ((EE[:,3]))
        print(world_init)
        x, y, z ,_ = world_init
        '''

        permuts_over_linspaces[idx] = sr

        print("i:", i, "init Q: ", Q_grid[idx], "sr: ",sr, "evals:", evals)
        print("\n")
        i+=1;

    permuts_over_linspaces=permuts_over_linspaces.flatten() #spectral radius
    # Flatten Q_grid to get all [q0, q1, ..., qN] configs
    Q_flat = Q_grid.reshape(-1, Q_grid.shape[-1])
    x, y, z = Q_flat[:, 0], Q_flat[:, 1], Q_flat[:,2]

    print("Generating Plot...")
    scatter = go.Scatter3d(x=x,y=y,z=z,mode='markers', marker=dict(size=8,color=permuts_over_linspaces, colorscale='plasma', colorbar=dict(title='spectral radius'), opacity=0.6))
    layout = go.Layout(
        scene=dict(
            xaxis_title='cartesian x',
            yaxis_title='cartesian y',
            zaxis_title='cartesian z'
        ),
        title='Spectral Radius in Cartesian Space',
        margin=dict(l=0,r=0,b=0,t=50)
    )
    fig=go.Figure(data=[scatter], layout=layout)

    fig.show()

def main():
    cur = [0, 0, sp.pi/2]
    des = [0,0,sp.pi/2]

    origin_world = sp.Matrix([[0],[0],[0],[1]])
    
    u_des,v_des,w_des = des
    #u_des, v_des, w_des = sp.symbols('u_des, v_des, w_des', real=True)
    reps_des=[(u,u_des), (v,v_des), (w, w_des)]

    ucur, vcur, wcur = cur
    reps_cur = [(u, ucur), (v, vcur), (w, wcur)]

    joints, EE =denavit_hartenburg_dylan_3dof(u,v,w)
    world_position = ((EE[:,3]))
    image_position = P * world_position

    #print("P:", P)
   # print("\nimage:", image_position, "\n")

    #DO NOT TOUCH THESE CAMERAS they were really onerous to place :-(
    cam1 = camera(0,0,0,[0,0,5], 5,5, 0, 0) #looks down directly at the scene, basically replicates the scene
    cam2 = camera(-sp.pi/2, 0, 0, [0,0,5], 5,5,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
    cameras=[cam1, cam2]
    
    print("World Pos:")
    print(world_position.subs(reps_cur))
    print("Image Pos Cam 1:")
    print(cam1.projectpoint(world_position.subs(reps_cur)))
    print("Image Pos Cam 2:")
    print(cam2.projectpoint(world_position.subs(reps_cur)))
    
    
    analytic_jacobian= compute_analytic_jac(cameras)
    dF = analytic_jacobian.subs(reps_des)
    print("dF:\n", dF, "\n")
    
    J = analytic_jacobian.subs(reps_cur)
    #print("J:", J)
    
    try:    
        B = J.pinv()
        #print("B:", B)
    except:
        print("error computing B")
        B=None
    
    I = sp.eye(3)

    A = (I - B*dF)

    #print("\nA:")
    #print(A)

    evals, sr = evals_and_sr(A)

    #print("\nevals:")
    #print(evals)
    print("\nsr:")
    print(sr)

    jointlimits = [[0,np.pi/2], [-np.pi/2,np.pi/2], [-np.pi/2,np.pi/2]]
    #plot(des, cameras, 8, jointlimits)

    #wireframe(joints.subs(reps_des))
    #print(position.subs(reps_des))

    #position is our analytical cartesian world point!
    #'''


main()
'''
Now we have the end effector of the robot and we also have the camera. 
If we only care about translation and not rotation, 
we can simply multiply our EE world point by our camera and get the projected point.

With this 
'''