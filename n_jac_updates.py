'''
May 26 2025

Plot how many Jacobian updates were needed for successfuly convergence: hopefully from this we can see WHICH regions are the ones with more updates.
hypothesis: getting IN/OUT of a nonlinear zone may be quite difficult. Ie if desired is in the center of a nonlinear zone, we may expect overall HIGH Jac iterations/updates needed.
'''

import numpy as np
import math
from spatialmath import SE3
from spatialmath.base import e2h, h2e
from machinevisiontoolbox import CentralCamera
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go
from scipy.spatial import Delaunay
import successive_mesh_refinement as smr



PI = np.pi


def fkin(q, ets: rtb.ETS):
    '''
    return the real world/actual position in R3 of the end effector based on the specified joint parameters, q (np array).
    '''
    T = ets.eval(q)       
    x=T[0][3];
    y=T[1][3];
    z=T[2][3];
    return np.array([x,y,z]);

def centraldiff_jacobian(currQ, desiredP, e: rtb.Robot.ets):
    '''
    Initialize and return a Jacobian through central differences, without use of a camera.'''
    epsilon=PI/16
     ###sanity check for our dimensions: 
    # let p=robot params, k=datapts for overdet system. 
    # each dP=1xd, each dQ=1xp. after generating overdet sys, 
    # DP=kxd, DQ=kxp, currJ.T=pxd... so currJ=dxp. 
    # Also, I is pxp
    # in this case, p=3, d=2
    p=currQ.shape[0]
    d=3 #d is world dimensions
    k=1 #for now its not overdetermined. NOTE: find out if Dylan's paper used overdet.
    currJT = np.zeros((p,d)) 
    I=np.identity(p)
    for i in range(p): #i is a specific robot param
        #the jacobian is first initialized with central differences. 
        '''print("INIT JAC: currQ: ")
        print(currQ)
        print(I[i])
        print("positive perturb", vs_fkin(e, currQ+epsilon*I[i], camera))
        print("negative perturb", vs_fkin(e, currQ-epsilon*I[i], camera))
        print("perturb diff",  (vs_fkin(e, currQ+epsilon*I[i], camera)) - (vs_fkin(e, currQ-epsilon*I[i], camera)))'''
        currJT[i] = (
                    (fkin(currQ+epsilon*I[i], e)) -
                    (fkin(currQ-epsilon*I[i], e))
                ) / (2*epsilon)
        #print("currJT i column", currJT[i])

    currJ=currJT.T
    return currJ

#inverse kinematics with constant jacobian: terminates when error is low enough, or when n=10
def constjac(currQ, desiredP, e : rtb.Robot.ets, mesh: smr.DelaunayMesh, simplex_mode=0): #uses a constant Jacobian.
    '''
    CONSTANT JACOBIAN for inverse kinematics. No camera involved yet. Returns resulting position when restol satisfied OR when n=MAXITER

    Steps:
    1. Project the currRP of the EE into the camera and use central differences to initialize the Jacobian. (initialization here and in uncalibrated_vs func are the same!)
    2. 

    '''
    i = 0

    currP=fkin(currQ, e) #projection of that real point through the camera
    errorP=desiredP-currP 
    error=np.linalg.norm(errorP)
    if error < TOLERANCE:
        return (currP) # return without any invkin, our ready pose IS the desired pose

    alpha=0.1

    print("currQ", currQ)
    curr_simplex = smr.find_simplex(currQ, mesh) #find which linear piece we are currently in
    J=centraldiff_jacobian(currQ, desiredP, e)

    corrQ =np.linalg.pinv(J) @ errorP #solve for change in theta 
    #print("corrQ:",corrQ)
    currQ=currQ+corrQ*alpha #alpha is our dampening factor
    ###after solvign for the new theta, we move there and check the error.
    currP=fkin(currQ, e)
    errorP=desiredP-currP #error is measured from projection img
    error=np.linalg.norm(errorP)

    i+=1
    while i< MAXITER and error>TOLERANCE:
        
        if simplex_mode:
            next_simplex= smr.find_simplex(currQ, mesh) #find which linear piece we are currently in\\
            print("currQ to be in next simplex:", currQ)
            print("iteration:",i,"simplexes:",curr_simplex, "VS", next_simplex)
            if curr_simplex!=next_simplex: #then we are in a new linear segement
                print("RECALIBRATE JACOBIAN.")
                print("J before:", J)
                J=centraldiff_jacobian(currQ, desiredP, e)
                print("J after:", J)
                curr_simplex=next_simplex #update curr_simplex   '''     
            
        currP  =  fkin(currQ, e) #get the current point...

        errorP = (desiredP - currP) #see our position error (vector)
        error = np.linalg.norm(errorP) #get the total error (scalar)

        corrQ = np.linalg.pinv(J) @ errorP #use the constant Jac and our current error to solve for corrQ

        currQ = currQ + corrQ*alpha
        i += 1  # update iteration variable

    return currP

def plotting(n, e, jointlimits: list, desiredP, mesh: smr.DelaunayMesh, simplex_mode:int):

    joint_ranges = [np.linspace(low, high, n) for (low, high) in jointlimits]
    grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)

    # preallocate error arrays (same shape as grid, but without dof)
    permuts_over_linspaces_shape = Q_grid.shape[:-1]
    permuts_over_linspaces = np.zeros(permuts_over_linspaces_shape)

    sum_error=0;

    i=0;
    for idx in np.ndindex(permuts_over_linspaces_shape): #iter over every permut over linspace of q0...qn 
                '''
                This for loop iterates over every pair/permutation in the linear spaces of our parameters,
                and returns the resulting point. 
                '''
                Q = Q_grid[idx] 
                resultP = constjac(currQ=Q, desiredP=desiredP, e=e, mesh=mesh, simplex_mode=simplex_mode)
                error=np.linalg.norm(desiredP - resultP) 
                sum_error+=error
                permuts_over_linspaces[idx] = error

                print("i:", i, "initial Q:",Q, "resultP:", resultP, "error:", error)
                i+=1;
    
    permuts_over_linspaces=permuts_over_linspaces.flatten()
    # Flatten Q_grid to get all [q0, q1, ..., qN] configs
    Q_flat = Q_grid.reshape(-1, Q_grid.shape[-1])
    x, y, z = Q_flat[:, 0], Q_flat[:, 1], permuts_over_linspaces


    print(permuts_over_linspaces)
    print("Generating Plot...")
    print("Sum of error: ", sum_error)
    scatter = go.Scatter3d(x=x,y=y,z=z,mode='markers', marker=dict(size=15,color=permuts_over_linspaces, colorscale='plasma', colorbar=dict(title='Total Error Non-Normalized'), opacity=0.2))
    layout = go.Layout(
        scene=dict(
            xaxis_title='q0',
            yaxis_title='q1',
            zaxis_title='error'
        ),
        title='Joint Space Error in 3D',
        margin=dict(l=0,r=0,b=0,t=50)
    )
    fig=go.Figure(data=[scatter], layout=layout)

    fig.show()

    if len(jointlimits) == 1: #1 DOF
        plt.plot(permuts_over_linspaces, 'o')
        plt.show()


TOLERANCE = 1e-3
MAXITER = 100
def main():
    l0=1;l1=1;l2=1;

    ets1dof = rtb.ET.Rz() * rtb.ET.tx(l0)
    joint_limits1dof = [(-np.pi/2, np.pi/2)]
    joint_limits1dof_full = [(-2*np.pi, 2*np.pi)]

    ets2dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rz() * rtb.ET.tx(l1) 
    joint_limits2dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2)]  # example for 2 DOF
    joint_limits2dof_full = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi)]

    ets3dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rx() * rtb.ET.tz(l1) * rtb.ET.Rx() * rtb.ET.tz(l2)
    joint_limits3dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]  # example for 2 DOF

    joint_limits = joint_limits2dof
    joint_limits_full = joint_limits2dof_full
    ets=ets2dof 

    camera = CentralCamera()
    robot = rtb.Robot(ets)

    mesh = smr.DelaunayMesh(1e-1, robot, camera, sparse_step=5, jointlimits=joint_limits_full)

    smr.create_sparsespace_chebyshev(mesh)
    smr.create_delaunaymesh_2DOF(mesh, 1)
    smr.calculate_simplices(mesh) 

    n=50
    simplex_mode=0
    plotting(n, ets, joint_limits, np.array([1,1,0]), mesh, simplex_mode)


if __name__ == '__main__':
    main()
