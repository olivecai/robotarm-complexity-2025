'''
2D Cartesian space.

After this, the next step is to work with a 3DOF arm.

June 10 2025
Start from a certain joint configuration and compute whatever inverse kinematics method to move to the desired goal.
Return the position error.
Compute the forward kinematics for each initial configuration.

For each initial position in forward kinematics point, plot the error.
'''

import numpy as np
import math
from spatialmath import SE3
from spatialmath.base import e2h, h2e
import machinevisiontoolbox as mvtb
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go
from scipy.spatial import Delaunay
import successive_mesh_refinement as smr
from common_robot_calculations import * 
from roboticstoolbox.models.DH import Puma560

PI = np.pi


def invkin(tolerance:int, maxiter: int, currQ, desiredP, e : rtb.Robot.ets, jointlimits : list, mesh: smr.DelaunayMesh, jacobian_method, simplex_mode, plot_traj=0): #uses a constant Jacobian.
    '''
    Inverse kinematics for constant Jacobian, simplices update, or every update, and supports simplices updates.
    No camera involved yet. Returns resulting position when restol satisfied OR when n=MAXITER
    '''
    simplices_visited=[]
    trajectory = []
    jac_updates=0;

    alpha = 1 #dampening factor
    if jacobian_method==1:
        J = centraldiff_jacobian(currQ, e)
    if jacobian_method==2 or jacobian_method==3:
        J=analytic_jacobian(currQ, e)
    if jacobian_method==4 or jacobian_method==5:
        J=mesh.mesh_jacobians[smr.find_simplex(currQ,mesh)]

    curr_simplex = smr.find_simplex(currQ, mesh)
    simplices_visited.append(curr_simplex)

    for i in range(maxiter):
        currP = fkin(currQ, e)
        errorP = desiredP - currP
        error = np.linalg.norm(errorP)

        trajectory.append(currQ.copy())

        #print(J,"\n")
        if error < tolerance:
            break

        if jacobian_method==3:
            J=analytic_jacobian(currQ, e)
            jac_updates+=1;

        if simplex_mode: #if not simplex mode then just use a constant jacobian the entire time
            next_simplex = smr.find_simplex(currQ, mesh)
            if next_simplex != curr_simplex:
                print("jac update at iter", i, "into simplex", next_simplex)
                jac_updates+=1;
                if jacobian_method==1:
                    J = centraldiff_jacobian(currQ, e)
                if jacobian_method==2:
                    J=analytic_jacobian(currQ, e)
                if jacobian_method==4 or jacobian_method==5:
                    J=mesh.mesh_jacobians[smr.find_simplex(currQ,mesh)]

                '''
                dynamic dampening:
                if we realize oscillatory motion is occuring, decrease alpha.
                '''
                if next_simplex in simplices_visited: #if we have seen this path before. Only applies in the case where we revisit one linear region: repeatedly iterating in the same simplex is unaffected.
                    alpha*=1

                    print("dampening coeff is now", alpha)

                curr_simplex = next_simplex
                simplices_visited.append(curr_simplex)

        corrQ = np.linalg.pinv(J) @ errorP
        
        currQ += alpha * corrQ

    return 1, currP, i, jac_updates

def calculate_error(camera: mvtb.CentralCamera, tolerance, maxiter, resolution, e, jointlimits: list, desiredP, mesh: smr.DelaunayMesh, jacobian_method, simplex_mode, plot_traj, plot_error):

    joint_ranges = [np.linspace(low, high, resolution) for (low, high) in jointlimits]

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
                Q = Q_grid[idx].copy()
                cartesian_initial_x = fkin(Q, e)[0]
                cartesian_initial_y = fkin(Q, e)[1]


#ICSDIFHSIDHFSOHUDFOUSHFDS PICK UP HEEERE
SDOJFSJNFJSDFBSIDHFSIHDFS PICK UP HERE TOMROROWWW!!!!!!!!!!!!!11


                if camera:
                    pass # TODO add camera ughhh we need to add the camera eventually but i keep getting sidetracked with other simulations  #success, resultP, iterations, jac_updates = vs_invkin(camera, tolerance=tolerance, maxiter=maxiter, currQ=Q, desiredP=desiredP, e=e, jointlimits=jointlimits, mesh=mesh, jacobian_method=jacobian_method, simplex_mode=simplex_mode)
                else:
                    success, resultP, iterations, jac_updates = invkin(tolerance=tolerance, maxiter=maxiter, currQ=Q, desiredP=desiredP, e=e, jointlimits=jointlimits, mesh=mesh, jacobian_method=jacobian_method, simplex_mode=simplex_mode)
                error=np.linalg.norm(desiredP - resultP) 
                sum_error+=error
                permuts_over_linspaces[idx] = error
                if not success and error>tolerance:
                    print("DEBUGGING")
                    permuts_over_linspaces[idx] = None

                print("i:", i, "init Q: ", Q_grid[idx], "fin Q:", Q, "result P:", resultP, "error:", error, "jac updates:", jac_updates)
                print("\n")
                i+=1;
    
    print("Sum of error: ", sum_error)

    if plot_error:
        permuts_over_linspaces=permuts_over_linspaces.flatten()
        # Flatten Q_grid to get all [q0, q1, ..., qN] configs
        Q_flat = Q_grid.reshape(-1, Q_grid.shape[-1])
        x, y, z = Q_flat[:, 0], Q_flat[:, 1], permuts_over_linspaces

        print("Generating Plot...")
        scatter = go.Scatter3d(x=x,y=y,z=z,mode='markers', marker=dict(size=8,color=permuts_over_linspaces, colorscale='plasma', colorbar=dict(title='Total Error Non-Normalized'), opacity=0.6))
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


def main():
    l0=1;l1=1;l2=1;

    ets1dof = rtb.ET.Rz() * rtb.ET.tx(l0)
    joint_limits1dof = [(-np.pi/2, np.pi/2)]
    joint_limits1dof_full = [(-2*np.pi, 2*np.pi)]

    ets2dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rz() * rtb.ET.tx(l1) 
    joint_limits2dof = [(0, np.pi), (-np.pi, np.pi)]  # example for 2 DOF
    joint_limits2dof_full = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi)]

    ets3dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rx() * rtb.ET.tz(l1) * rtb.ET.Rx() * rtb.ET.tz(l2)
    joint_limits3dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits3dof_full = [(-2*np.pi/2, 2*np.pi/2), (-2*np.pi/2, 2*np.pi/2) , (-2*np.pi/2, 2*np.pi/2)]

    puma = Puma560()
    ets_puma = (puma.ets())  # shows ETS version
    joint_limits_puma= [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_puma_full = [(-2*np.pi/2, 2*np.pi/2), (-2*np.pi/2, 2*np.pi/2) , (-2*np.pi/2, 2*np.pi/2),(-2*np.pi/2, 2*np.pi/2), (-2*np.pi/2, 2*np.pi/2) , (-2*np.pi/2, 2*np.pi/2)]

    joint_limits = joint_limits2dof
    joint_limits_full = joint_limits2dof_full
    ets=ets2dof

    robot = rtb.Robot(ets)

    ############  toggle buttons ########################################

    #CAMERA PARAMS
    f = 8*1e-3     # focal length in metres
    rho = 10*1e-6  # pixel side length in metres
    u0 = 500       # principal point, horizontal coordinate
    v0 = 500       # principal point, vertical coordinate
    camera : mvtb.CentralCamera = init_camera(f, rho, u0, v0, 1000)
    camera=None

    #MESH PARAMS
    tolerance = 1e-3
    maxiter = 100
    resolution=5
    chebyshev = 0 #chebyshev seems to consistently result in a tiny bit more error than equidistant...

    #PLOTTING PARAMS
    desiredP = np.array([1,1,0])
    Q = np.array([2.5,2.5])
    simplex_mode=0
    #### JACOBIAN METHODS ####
    # 1 central diff, 2 analytic simplex, 3 analytic every update (best possible)
    # 4 central differences assigned to each simplex, 5 analytic assigned to each simplex. 
    jacobian_method=1
    #####################################################################

    #meshing should perhaps not use the camera at all.
    #TODO: Ask about camera in mesh.
    mesh = smr.DelaunayMesh(1e-1, robot, camera, sparse_step=10, jointlimits=joint_limits_full)

    print("Creating Delaunay Mesh...")
    if chebyshev:
        smr.create_sparsespace_chebyshev(mesh)
    else: 
        smr.create_sparsespace(mesh)
    #smr.create_delaunaymesh_2DOF(mesh, 1) #NOTE: this line is NOT necessary for computations, it is simply to show the viewer our plot.
    smr.calculate_simplices(mesh) #this actually calculates the simplices we need.

    #dont toggle below
    if jacobian_method==2:
        simplex_mode=1
    if jacobian_method==3:
        simplex_mode=0
    if jacobian_method==4:
        smr.create_mesh_jacobians(mesh, ets, 1)
    if jacobian_method==5:
        smr.create_mesh_jacobians(mesh,ets,2)

    calculate_error(camera, tolerance, maxiter, resolution, ets, joint_limits, desiredP, mesh, jacobian_method, simplex_mode, 0, 1)
   
if __name__ == '__main__':
    main()
  
