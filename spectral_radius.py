'''
June 4 2025

Exploring spectral radius and singular Jacobians.

############# OVERVIEW #############
The spectral radius (referred to as p here) can tell us how stable a system is:
p < 1 : matrix converges
else, matrix diverges.

a "convergent matrix" converges to the zero matrix under matrix exponentiation.
### read about convergent matrixes here: https://en.wikipedia.org/wiki/Convergent_matrix 

##### ASIDE:
# Note that the Numerical Methods in Computing Science textbook uses the notation phi(x) = x-f(x)*g(x) where x=Q, and phi=g, f=F, and their g() is chosen to be B here. (We are using Newton's Method)
Qn+1 = g(Qn), where Qk is joint parameters at iteration k, and g(...) is the update function, as defined below:
gn(Qn) = Qn - BF(Qn)
    B  is the J_inverse calculated at the starting position through central differences <==> 
    F  is F(Qn) = current-fkin(Qn) = current-desired, where current is a CONSTANT and fkin(Q*) is in symbols, so upon calculating dF, we obtain what may as well be the Jacobian for the desired position without any scaling.
In my code I will simply have F be the Jacobian of the forward kinematics of the desired position, since the displacement doesnt create a difference anyway. 
    
in Newton's method, we have successful convergence when B @ (del F(Qn) / del Qn) == I:
del gn / del Qn = I - B @ (del F(Qn) / del Qn)

g represewnts joint space error : is it always decreasing?


For ease in notation, we will refer to del gn / del Qn as matrix 'A'.
We will also refer to del F(Qn) / del Qn as matrix 'dF'
Matrix A is the Jacobian of the update function, gn. 
    It tells us how small changes in the current guess Qn affect the next guess.
    
############# EXECUTION #############
Simple experiment:
Using symbolic language libraries, get the derivatives and Jacobians for a single point that we KNOW converges with a constant Jacobian.
Get the spectral radius.
Repeat for a starting point that does NOT converge.
Repeat for a starting point which may pass through a singularity.
Repeat for a starting point at a singularity.

Expanding experiements:
Look into the joint spectral radius over a trajectory.
Focus on what the spectral radius can tell us that the Jacobian cannot 


use sym lang to calc A instead of using Dylans approx --> define a g(q) and take its partial derivatives wrt q

sybolic eigenvalues -> look at thr landscape of the evals -> expect near singularities, shoot up to inf 


Okay so if dF is based on a constant position, then lets use only symbols... and see where we can change our constant position and desired position so that we are guaranteed convergence...


'''


import sympy as sp
import successive_mesh_refinement as smr
import common_robot_calculations as crc
import numpy as np
import roboticstoolbox as rtb
import machinevisiontoolbox as mvtb
import matplotlib.pyplot as plt
import plotly.graph_objects as go

        
def spectral_radius(u_, v_, mesh, ets, camera, u0, v0):
    '''
    
    '''
     # u and v <==> theta1 and theta2
    u = sp.Symbol('u', real=True)
    v = sp.Symbol('v', real=True)
    # l1 and l2 
    l1 = sp.Symbol('l1', positive=True)
    l2 = sp.Symbol('l2', positive=True)
    l1=1;l2=1; #for ease let's just have lengths be 1

    #declare the forward kin functions
    f1 = l1 * sp.cos(u) + l2 * sp.cos(u+v)
    f2 = l1 * sp.sin(u) + l2 * sp.sin(u+v)

    I = sp.eye(2) #hardcoded to 2 for now, while we run experiments on the 2DOF planar arm
    f = sp.Matrix([f1, f2]) #forward kinematics matrix
    
    reps_pos = [(u, u_), (v, v_)]
    reps_des = [(u, u0), (v, v0)]
    
    #We have a few options to get B here...
    # NOTE that B uses the starting position.....
    # While F(Qn) uses the pure desired joint configuratinon.
    mode=3
    if mode==1:
        J = f.jacobian([u,v]) #ANALYTIC jacobian of the forward kinematics function
        #J=J.subs(reps_pos)
        try:
            B = J.inv() #ANALYTIC
        except:
            B=None
    if mode==2: #simplex designated jacobian
        J=np.delete(mesh.mesh_jacobians[smr.find_simplex(np.array([u_,v_]),mesh)], 2, axis=0)
        try:
            B=np.linalg.inv(J)
        except:
            B=None
    if mode==3:
        J=np.delete(crc.centraldiff_jacobian(np.array([u_,v_]),ets), 2, axis=0)
        try:
            B=np.linalg.inv(J)
        except:
            B=None
    print(J)
        

    if B is None:
        sr = None

    else:
        F = f 
        print(F)
        dF = F.jacobian([u,v])
       
        A = I - B*dF

        print(A)

        sr=calculate_spectral_radius(A.subs(reps_des)) 
    
    print(sr)

    return sr

      
def analytic_spectral_radius(ets, camera):
    '''
    
    '''
    # u and v <==> theta1 and theta2
    u = sp.Symbol('u', real=True)
    v = sp.Symbol('v', real=True)
    # desired u and v <==> theta1 and theta2
    u_des = sp.Symbol('u_des', real=True)
    v_des = sp.Symbol('v_des', real=True)

    reps_des = [(u, u_des), (v, v_des)]

    # l1 and l2 
    l1 = sp.Symbol('l1', positive=True)
    l2 = sp.Symbol('l2', positive=True)
    l1=1;l2=1; #for ease let's just have lengths be 1

    #declare the forward kin functions
    f1 = l1 * sp.cos(u) + l2 * sp.cos(u+v)
    f2 = l1 * sp.sin(u) + l2 * sp.sin(u+v)

    I = sp.eye(2) #hardcoded to 2 for now, while we run experiments on the 2DOF planar arm
    f = sp.Matrix([f1, f2]) #forward kinematics matrix
    
    J = f.jacobian([u,v]) #ANALYTIC jacobian of the forward kinematics function
    try:
        B = J.inv() #ANALYTIC
    except:
        print("error")
        B=None
    print("Jacobian of initial position:\n", J)
        

    if B is None:
        sr = None

    else:
        dF = J.subs(reps_des)
        print("Jacobian of desired position:\n", J)

        A = I - B*dF

        print("del gn / del qn:\n", A)

        sr=calculate_spectral_radius(A) 
    
    print("spectral radius: ",sr)

    return sr

def calculate_spectral_radius(A):
    # Get the eigenvalues
    eigenvals = A.eigenvals()  # returns dict: {eigenvalue: multiplicity}
    # Compute the spectral radius (max absolute eigenvalue)
    return sp.Max(*[sp.Abs(lam) for lam in eigenvals.keys()])

def linearspace(resolution, jointlimits, mesh, ets, camera, u0, v0):

    joint_ranges = [np.linspace(low, high, resolution) for (low, high) in jointlimits]

    grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)

    # preallocate error arrays (same shape as grid, but without dof)
    permuts_over_linspaces_shape = Q_grid.shape[:-1]
    permuts_over_linspaces = np.zeros(permuts_over_linspaces_shape)

    i=0
    for idx in np.ndindex(permuts_over_linspaces_shape): #iter over every permut over linspace of q0...qn 
            '''
            This for loop iterates over every pair/permutation in the linear spaces of our parameters.
            '''
            Q = Q_grid[idx].copy()
            sr = spectral_radius(Q[0], Q[1], mesh, ets, camera, u0, v0)
            permuts_over_linspaces[idx] = sr

            print("i:", i, "init Q: ", Q_grid[idx], "fin Q:", Q, "spectral radius:", sr)
            i+=1;
    
    permuts_over_linspaces=permuts_over_linspaces.flatten()
    # Flatten Q_grid to get all [q0, q1, ..., qN] configs
    Q_flat = Q_grid.reshape(-1, Q_grid.shape[-1])
    x, y, z = Q_flat[:, 0], Q_flat[:, 1], permuts_over_linspaces

    print("Generating Plot...")
    scatter = go.Scatter3d(x=x,y=y,z=z,mode='markers', marker=dict(size=6,color=permuts_over_linspaces, colorscale='plasma', colorbar=dict(title='Total Error Non-Normalized'), opacity=0.6))
    layout = go.Layout(
        scene=dict(
            xaxis_title='q0',
            yaxis_title='q1',
            zaxis_title='spectral radius'
        ),
        title='2DOF spectral radius',
        margin=dict(l=0,r=0,b=0,t=50)
    )
    fig=go.Figure(data=[scatter], layout=layout)

    fig.show()

def symbols_comparison():
    '''
    I need to see the difference between Newton's Method and the optimization method I'm doing now because theya re different but I was undert the assumption that both were Newton's method up till now.
    '''
    # current u and v <==> theta1 and theta2
    u = sp.Symbol('u', real=True)
    v = sp.Symbol('v', real=True)
    q = sp.Matrix([u,v])

    # desired u and v <==> theta1 and theta2
    u0 = sp.Symbol('u0', real=True)
    v0 = sp.Symbol('v0', real=True)

    # l1 and l2 
    l1 = sp.Symbol('l1', positive=True)
    l2 = sp.Symbol('l2', positive=True)

    #declare the forward kin functions
    f1 = l1 * sp.cos(u) + l2 * sp.cos(u+v)
    f2 = l1 * sp.sin(u) + l2 * sp.sin(u+v)

    I = sp.eye(2) #hardcoded to 2 for now, while we run experiments on the 2DOF planar arm
    f = sp.Matrix([f1, f2]) #forward kinematics matrix

    J = f.jacobian([u,v]) #ANALYTIC jacobian of the forward kinematics function
    B = J.inv() #ANALYTIC

    F = f #TODO: clarify. before, F was the cartesian space residual. Now, it is the desired cartesian position....(???)
    dF = F.jacobian([u,v])

    next_q = q - B*F


def main():

  

    l1=1;l2=1;

    ets2dof = rtb.ET.Rz() * rtb.ET.tx(l1) * rtb.ET.Rz() * rtb.ET.tx(l2) 
    joint_limits2dof = [(0, np.pi), (-np.pi, np.pi)]  # example for 2 DOF
    joint_limits2dof_full = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi)]
    jointlimits = joint_limits2dof
    joint_limits_full = joint_limits2dof_full
    ets=ets2dof
    robot = rtb.Robot(ets)
    camera=None 

    mesh = smr.DelaunayMesh(1e-1, robot, camera, sparse_step=10, jointlimits=joint_limits_full)
    smr.create_sparsespace(mesh)
    #smr.create_delaunaymesh_2DOF(mesh, 1) #NOTE: this line is NOT necessary for computations, it is simply to show the viewer our plot.
    smr.calculate_simplices(mesh) #this actually calculates the simplices we need.
    smr.create_mesh_jacobians(mesh, ets, 1)
    
    #starting position
    u_ = 0
    v_ = sp.pi/2

    #desired position
    u0=sp.pi/2
    v0=-sp.pi/2

    resolution=20
    linearspace(resolution, jointlimits, mesh, ets, camera, u0, v0)  

    #spectral_radius(u_, v_, mesh, ets, camera, u0, v0)
    #analytic_spectral_radius(ets, camera)

#So, there are two things we need to consider: estimating both B and dF
    




main()



