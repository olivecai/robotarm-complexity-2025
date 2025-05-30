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
from common_robot_calculations import fkin, centraldiff_jacobian, analytic_jacobian

PI = np.pi

#inverse kinematics with constant jacobian: terminates when error is low enough, or when n=10
def invkin_constjac(tolerance:int, maxiter: int, currQ, desiredP, e : rtb.Robot.ets, mesh: smr.DelaunayMesh, jacobian_method, simplex_mode, plot_traj=0): #uses a constant Jacobian.
    '''
    CONSTANT JACOBIAN for inverse kinematics. No camera involved yet. Returns resulting position when restol satisfied OR when n=MAXITER
    '''
    
    trajectory = []
    jac_updates=0;

    alpha = 0.05  #dampening factor
    if jacobian_method==1:
        J = centraldiff_jacobian(currQ, e)
    if jacobian_method==2 or jacobian_method==3:
        J=analytic_jacobian(currQ, e)
    if jacobian_method==4 or jacobian_method==5:
        J=mesh.mesh_jacobians[smr.find_simplex(currQ,mesh)]

    curr_simplex = smr.find_simplex(currQ, mesh)

    for i in range(maxiter):
        currP = fkin(currQ, e)
        errorP = desiredP - currP
        error = np.linalg.norm(errorP)

        trajectory.append(currQ.copy())

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

                curr_simplex = next_simplex

        corrQ = np.linalg.pinv(J) @ errorP
        #currQ=currQ.copy() #comment to modify currQ directly, uncomment to create a copy each time !!!!!
        currQ += alpha * corrQ

    if plot_traj: #animate trajectory, very fast
        e.plot(np.array(trajectory), block=False)
        plt.close('all')
        slider_robot_trajectory(np.array(trajectory))

    return currP, i, jac_updates

def calculate_error(tolerance, maxiter, resolution, e, jointlimits: list, desiredP, mesh: smr.DelaunayMesh, jacobian_method, simplex_mode, plot_traj, plot_error):

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
                resultP, iterations, jac_updates = invkin_constjac(tolerance=tolerance, maxiter=maxiter, currQ=Q, desiredP=desiredP, e=e, mesh=mesh, jacobian_method=jacobian_method, simplex_mode=simplex_mode)
                error=np.linalg.norm(desiredP - resultP) 
                sum_error+=error
                permuts_over_linspaces[idx] = error

                print("i:", i, "init Q: ", Q_grid[idx], "fin Q:", Q, "result P:", resultP, "error:", error, "jac updates:", jac_updates)
                i+=1;
    
    print("Sum of error: ", sum_error)

    if plot_error:
        permuts_over_linspaces=permuts_over_linspaces.flatten()
        # Flatten Q_grid to get all [q0, q1, ..., qN] configs
        Q_flat = Q_grid.reshape(-1, Q_grid.shape[-1])
        x, y, z = Q_flat[:, 0], Q_flat[:, 1], permuts_over_linspaces

        print("Generating Plot...")
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

def plot_robot_trajectory(tolerance, maxiter, Q, desiredP, ets, mesh, jacobian_method, simplex_mode, plot_traj_mode):
    robot=rtb.Robot(ets)
    initQ=Q.copy()
    resultP, iterations, jac_update_count = invkin_constjac(tolerance=tolerance, maxiter=maxiter, currQ=Q, desiredP=desiredP, e=ets, mesh=mesh, jacobian_method=jacobian_method, simplex_mode=simplex_mode, plot_traj=plot_traj_mode)
    error=np.linalg.norm(desiredP - resultP) 
    print("Plot Trajectory:\ninitial Q: ", initQ, "result Q:", Q, "resultP:", resultP, "error:", error, "jacobian updates:" ,jac_update_count)

def slider_robot_trajectory(traj):
    x, y = traj[:, 0], traj[:, 1]

    # Create frames: one per timestep
    frames = [
        go.Frame(
            data=[
                go.Scatter(x=x[:k+1], y=y[:k+1], mode='lines+markers',
                           line=dict(color='blue'), marker=dict(size=6))
            ],
            name=str(k)
        )
        for k in range(len(x))
    ]

    layout = go.Layout(
        title="Joint Trajectory (q0 vs q1)",
        xaxis=dict(title="q0"),
        yaxis=dict(title="q1"),
        updatemenus=[dict(
            type="buttons",
            showactive=False,
            buttons=[
                dict(label="Play", method="animate", args=[None, {"frame": {"duration": 300}, "fromcurrent": True}]),
                dict(label="Pause", method="animate", args=[[None], {"frame": {"duration": 0}, "mode": "immediate"}])
            ]
        )],
        sliders=[{
            "steps": [{
                "args": [[f.name], {"frame": {"duration": 0, "redraw": True},
                                    "mode": "immediate"}],
                "label": f.name,
                "method": "animate"
            } for f in frames],
            "transition": {"duration": 0},
            "x": 0, "y": -0.1,
            "currentvalue": {"prefix": "Iteration: "}
        }],
    )

    fig = go.Figure(
        data=[go.Scatter(x=[x[0]], y=[y[0]], mode="lines+markers")],
        layout=layout,
        frames=frames
    )

    fig.show()

def main():
    l0=1;l1=1;l2=1;

    ets1dof = rtb.ET.Rz() * rtb.ET.tx(l0)
    joint_limits1dof = [(-np.pi/2, np.pi/2)]
    joint_limits1dof_full = [(-2*np.pi, 2*np.pi)]

    ets2dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rz() * rtb.ET.tx(l1) 
    joint_limits2dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2)]  # example for 2 DOF
    joint_limits2dof_full = [(-np.pi, np.pi), (-np.pi, np.pi)]

    ets3dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rx() * rtb.ET.tz(l1) * rtb.ET.Rx() * rtb.ET.tz(l2)
    joint_limits3dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]  # example for 2 DOF

    joint_limits = joint_limits2dof
    joint_limits_full = joint_limits2dof_full
    ets=ets2dof 

    camera = CentralCamera()
    robot = rtb.Robot(ets)

    mesh = smr.DelaunayMesh(1e-1, robot, camera, sparse_step=2, jointlimits=joint_limits_full)

    ########  toggle buttons #######
    tolerance = 1e-3
    maxiter = 100
    resolution=15
    desiredP = np.array([1,1,0])
    chebyshev = 0 #chebyshev seems to consistently result in a tiny bit more error than equidistant...
    Q = np.array([float(0.8973979), float(0)])
    plot_certain_trajectory=0
    simplex_mode=1
    #JACOBIAN METHODS:
    # 1 central diff, 2 analytic simplex, 3 analytic every update (best possible)
    # 4 central differences assigned to each simplex, 5 analytic assigned to each simplex. 
    jacobian_method=1


    print("Creating Delaunay Mesh...")
    if chebyshev:
        smr.create_sparsespace_chebyshev(mesh)
    else: 
        smr.create_sparsespace(mesh)
    smr.create_delaunaymesh_2DOF(mesh, 1) #NOTE: this line is NOT necessary for computations, it is simply to show the viewer our plot.
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

    if not plot_certain_trajectory:
        print("Plotting error...")
        calculate_error(tolerance, maxiter, resolution, ets, joint_limits, desiredP, mesh, jacobian_method, simplex_mode, 0, 1)
    else:
        plot_robot_trajectory(tolerance, maxiter, Q.copy(), desiredP, ets, mesh, jacobian_method, simplex_mode, 1)


if __name__ == '__main__':
    main()
  
