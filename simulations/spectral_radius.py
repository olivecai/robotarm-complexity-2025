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

from sympy import sin, cos, pi
        
def sr_3dof(i0,i1,i2, ets, camera, d0,d1,d2, alpha):
        # u and v <==> theta1 and theta2
    t0,t1,t2 = sp.symbols('t(0:3)', real=True)
    # desired u and v <==> theta1 and theta2
    reps_init = [(t0, i0), (t1, i1), (t2,i2)]
    reps_des = [(t0, d0), (t1, d1), (t2,d2)]


    #declare the forward kin functions
    #f1 = l1 * sp.cos(u) + l2 * sp.cos(u+v) #TODO: collect on f1, f2
    #f2 = l1 * sp.sin(u) + l2 * sp.sin(u+v)
    f= sp.Matrix([[-0.3*sin(t1)*sin(t2)*cos(t0) + 0.3*cos(t0)*cos(t1)*cos(t2) + 0.55*cos(t0)*cos(t1)],
              [-0.3*sin(t0)*sin(t1)*sin(t2) + 0.3*sin(t0)*cos(t1)*cos(t2) + 0.55*sin(t0)*cos(t1)], 
              [0.3*sin(t1)*cos(t2) + 0.55*sin(t1) + 0.3*sin(t2)*cos(t1)]])
    
    I = sp.eye(3) #hardcoded to 3 for now, while we run experiments on the 3DOF

    
    J = f.jacobian([t0,t1,t2]) #ANALYTIC jacobian of the forward kinematics function
    try:
        B = J.inv() #ANALYTIC
        B=B.subs(reps_init)
    except:
        print("error")
        B=None
    print("Jacobian of initial position:\n", J)
        

    if B is None:
        sr = None
        evals=None

    else:
        print(reps_des)
        dF = J.subs(reps_des)
        print("Jacobian of desired position:\n", J)

        A = I - B*dF

        print("del gn / del qn:\n", A)

        evals, sr=evals_and_sr(A)
    
    print("spectral radius: ",sr)
    print("evals:", evals)
    

    return evals, sr
    

def spectral_radius(u_, v_, ets, camera, u0, v0, alpha):
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
    f1 = l1 * sp.cos(u) + l2 * sp.cos(u+v)  # x coord
    f2 = l1 * sp.sin(u) + l2 * sp.sin(u+v)  # y coord

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
    if mode==3:
        J=np.delete(crc.centraldiff_jacobian(np.array([u_,v_]),ets), 2, axis=0)
        try:
            B=np.linalg.inv(J)
        except:
            B=None
    #print(J)
        

    if B is None:
        sr = None
        evals=None

    else:
        F = f 
        #print(F)
        dF = F.jacobian([u,v])

        A = I - B*dF * alpha

        #print(A)

        A=A.subs(reps_des)
        #print(u0, v0)
        #print(u_, v_)
        #print(A)
        evals, sr=evals_and_sr(A)
    
    #print("spectral radius: ",sr)
    #print("evals:", evals)

    return evals, sr
      
def analytic_spectral_radius(ets, camera):
    '''
    
    '''
    # u and v <==> theta1 and theta2
    t0,t1,t2 = sp.symbols('t(0:3)', real=True)
    # desired u and v <==> theta1 and theta2
    u0,u1,u2 = sp.symbols('u(0:3)', real=True)

    reps_des = [(t0, u0), (t1, u1), (t2,u2)]

    #declare the forward kin functions
    #f1 = l1 * sp.cos(u) + l2 * sp.cos(u+v) #TODO: collect on f1, f2
    #f2 = l1 * sp.sin(u) + l2 * sp.sin(u+v)
    f= sp.Matrix([[-0.3*sin(t1)*sin(t2)*cos(t0) + 0.3*cos(t0)*cos(t1)*cos(t2) + 0.55*cos(t0)*cos(t1)],
              [-0.3*sin(t0)*sin(t1)*sin(t2) + 0.3*sin(t0)*cos(t1)*cos(t2) + 0.55*sin(t0)*cos(t1)], 
              [0.3*sin(t1)*cos(t2) + 0.55*sin(t1) + 0.3*sin(t2)*cos(t1)]])


    I = sp.eye(3) #hardcoded to 3 for now, while we run experiments on the 3DOF

    
    J = f.jacobian([t0,t1,t2]) #ANALYTIC jacobian of the forward kinematics function
    try:
        B = J.inv() #ANALYTIC
    except:
        print("error")
        B=None
    print("Jacobian of initial position:\n", J)
        

    if B is None:
        sr = None
        evals=None

    else:
        dF = J.subs(reps_des)
        print("Jacobian of desired position:\n", J)

        A = I - B*dF

        print("del gn / del qn:\n", A)

        evals, sr=evals_and_sr(A)
    
    print("spectral radius: ",sr)
    print("evals:", evals)

    

    return evals, sr


def evals_and_sr(A):
    
    # Get the eigenvalues
    try:
        eigenvals = A.eigenvals()  # returns dict: {eigenvalue: multiplicity}
        # Compute the spectral radius (max absolute eigenvalue)
        return list(eigenvals.keys()), sp.Max(*[sp.Abs(lam) for lam in eigenvals.keys()])
    except:
        return None, None

def products_of_spectral_radii(resolution, jointlimits, ets, camera):
    '''
    make a linearspace of desired positions,and calculate the corresponding spectral radius plot, where the spectral radius of each initial joint configuration wrt to the desired position is shown...
    then we perform an element wise operation on each entry...
    '''

    joint_ranges = [np.linspace(low, high, resolution) for (low, high) in jointlimits]

    grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)

    # preallocate error arrays (same shape as grid, but without dof)
    permuts_over_linspaces_shape = Q_grid.shape[:-1]

    permuts_over_linspaces = np.ones(permuts_over_linspaces_shape).flatten()
    print(permuts_over_linspaces)


    for idx in np.ndindex(permuts_over_linspaces_shape): #iter over every permut over linspace of q0...qn 
        Q = Q_grid[idx].copy()
        spectral_radii_map = linearspace(resolution, jointlimits, ets, camera, Q[0], Q[1], 1) #get the eigvals
        for i in range(len(permuts_over_linspaces)):
            if spectral_radii_map[i] is not None:
                permuts_over_linspaces[i] += spectral_radii_map[i]
    # Flatten Q_grid to get all [q0, q1, ..., qN] configs
    Q_flat = Q_grid.reshape(-1, Q_grid.shape[-1])
    x, y, z = Q_flat[:, 0], Q_flat[:, 1], permuts_over_linspaces

    print("Generating Plot...")
    scatter = go.Scatter3d(x=x,y=y,z=z,mode='markers', marker=dict(size=6,color=permuts_over_linspaces, colorscale='plasma', colorbar=dict(title='joint sr but its not actually joint'), opacity=0.6))
    layout = go.Layout(
        scene=dict(
            xaxis_title='q0',
            yaxis_title='q1',
            zaxis_title='joint sr'
        ),
        title='2DOF joint sr',
        margin=dict(l=0,r=0,b=0,t=50)
    )
    fig=go.Figure(data=[scatter], layout=layout)

    fig.show()


def linearspace(resolution, jointlimits, ets, camera, u0, v0, w0, mode):
    '''
    if mode ==1, see the spectral radius ranges
    if mode ==2, have a binary IS EIGVAL COMPLEX OR NOT (complex=1, real=0)
    '''
    alpha=1

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
            if mode ==1 or mode ==2:
                evals, sr = spectral_radius(Q[0], Q[1], ets, camera, u0, v0, alpha) #ALPHA IS HARDCODED WITHIN THIS FUNCTION RN
            
            if mode==1:
                permuts_over_linspaces[idx] = sr
                zaxistitle='spectral radius'
            if mode==2:
                zaxistitle='complex==1, real==0'
                if evals==None:
                    permuts_over_linspaces[idx] = None
                elif (sp.simplify(evals[0]).is_real):
                    permuts_over_linspaces[idx] = 0 #REAL
                else:
                    permuts_over_linspaces[idx] = 1 #COMPLEX

            if mode ==3:
                evals, sr = sr_3dof(Q[0],Q[1],Q[2], ets, camera, u0,v0,w0, alpha)


            #print("i:", i, "init Q: ", Q_grid[idx], "fin Q:", Q, "spectral radius:", sr)
            i+=1;
    
   
    # Flatten the data
    Q_flat = Q_grid.reshape(-1, Q_grid.shape[-1])
    x = Q_flat[:, 0]
    y = Q_flat[:, 1]
    z = np.zeros_like(x)  # use z=0 for a flat 3D surface if you just want color as "4th" dim
    c = permuts_over_linspaces.flatten()

    # Create 3D scatter plot
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    sc = ax.scatter(x, y, z, c=c, cmap='plasma', s=20)

    # Add colorbar
    cbar = plt.colorbar(sc, ax=ax)
    cbar.set_label('Total Error Non-Normalized')

    # Axis labels and title
    ax.set_xlabel('q0')
    ax.set_ylabel('q1')
    ax.set_zlabel('')  # Empty or something like 'dummy z'
    ax.set_title(f'2DOF {zaxistitle}')

    plt.tight_layout()
    plt.show()

    return permuts_over_linspaces

def spectral_radius_wrt_damping(u_, v_, ets, camera, u0, v0):
    '''
    This function plots how each initial joint configuration VS a goal configuration performs 
    '''
    u_=0.01
    v_=0.01
    #print("u_, v_:", u_, v_)
    #print("u0, v0:", u0, v0)
    points=[]
    dampings = np.linspace(1, 1000, 100)
    for damping in dampings:
        alpha = 1/damping
        #print("alpha:", alpha)
        eigval, sr = spectral_radius(u_, v_, ets, camera, u0, v0, alpha)
        print(eigval) # lim alpha -> 0 corresponds with the eigenvalues becoming 0.
        print(sr) # the spectral radius converges to 1. But do we necessarily converge? Well, technically, we get so close to 1 that it is "inconclusive" behaviour, so it may or may not. which is why we see more convergence when we dampen. # #maybe i should just go eat lunch right now.
        # actually interestingly enough, for initial position (0.5235988, 0.1) we dip down below 1 to around 0.9091 with alpha = 0.125, and then start creeping back up to 1! So damping can totally change the game 
        # so dampening wont make it NOT converge for any initial joint position.
        # perfect convergence --> eigvals == 0,

        points.append(sr)
    plt.plot(points, 'o')
    plt.yscale('log')
    plt.show()
    

def plot(u_des, v_des):
    u_vals = np.linspace(-np.pi, np.pi, 100)
    v_vals = np.linspace(0, np.pi, 100)
    U, V = np.meshgrid(u_vals, v_vals)

def main():

    l1=1;l2=1;

    ets2dof = rtb.ET.Rz() * rtb.ET.tx(l1) * rtb.ET.Rz() * rtb.ET.tx(l2) 
    joint_limits2dof = [(0, np.pi), (-np.pi, np.pi)]  # example for 2 DOF
    joint_limits2dof_full = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi)]

    ets_dylan = rtb.ET.Rz() * rtb.ET.Ry(np.pi/2) * rtb.ET.Rz(np.pi) * rtb.ET.Ry() * rtb.ET.tz(0.55) * rtb.ET.Ry() * rtb.ET.tz(0.30)
    #joint_limits_dylan = [(0, np.pi/2), (0, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_dylan = [(-np.pi, np.pi), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_full_dylan = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi) , (-2*np.pi, 2*np.pi)]
    dofdylan = ets_dylan, joint_limits_dylan, joint_limits_full_dylan


    ets, jointlimits, _ = dofdylan 
    robot = rtb.Robot(ets)
    camera=None 
    
    #starting position
    u_ = 0
    v_ = 0

    #desired position
    u0=0.0
    v0=sp.pi/2

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

    resolution=2
    mode=3
    spectral_radii= linearspace(resolution, jointlimits, ets, camera, u0, v0, None, mode)  
    alpha=1
    sr_3dof(2,2,2, ets, camera, 1,1,1,1)
    #evals, sr= spectral_radius(u_, v_, ets, camera, u0, v0, alpha)
    #sr = analytic_spectral_radius(ets, camera)

    #products_of_spectral_radii(resolution, jointlimits, ets, camera)

    #spectral_radius_wrt_damping(u_, v_, ets, camera, u0, v0)

    #analytic spectral radius:
    '''
    Max(
        Abs(
                (-2*sin(v) 
                + sin(-u + u_des + v_des) 
                + sin(u - u_des + v))   
                /(2*sin(v)) 
            - sqrt(         
                -2*cos(v - v_des) + cos(v + v_des) 
                - cos(-2*u + 2*u_des + 2*v_des)/2 
                - cos(2*u - 2*u_des + 2*v)/2 
                + cos(2*u - 2*u_des + v - v_des) 
                + 1)
                /(2*sin(v)
                )
        ), 

        Abs(
                (-2*sin(v)
                + sin(-u + u_des + v_des) 
                + sin(u - u_des + v))
                /(2*sin(v)) 
            + sqrt(
                -2*cos(v - v_des) + cos(v + v_des) 
                - cos(-2*u + 2*u_des + 2*v_des)/2 
                - cos(2*u - 2*u_des + 2*v)/2 
                + cos(2*u - 2*u_des + v - v_des) 
                + 1)
                /(2*sin(v)
                )
            )
        )
        
        If we have analytic invkin equation, then we can replace each u and v subs analytic form. Or, analytic fkin, with sym equations, collect 'collect' the fkin form
        It would be nice if collect: if equation is the equation for a circle

        From Dylan's numerical experiemnts, seems to have a lot of circle shape going on 

    Eigenvalues are conjugates of each other and when sin(v)==0, ie v=k*pi where k is any integer, 
    we may expect no convergence. This makes sense, ... either we are folded on ourself or outstretched.
    
    Can we make basins of attraction and compare that to our spectral radius plots?
    
    June 11:
    Best case when eigenvalues are both real, same, and negative. Does this ever happen?
    '''


#So, there are two things we need to consider: estimating both B and dF
    




main()



