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
from common_robot_calculations import * 
from roboticstoolbox.models.DH import Puma560

PI = np.pi

def is_close_to_singular(currQ, e: rtb.Robot.ets):
    '''
    Return 1 if SINGULAR
    Return 0 if NOT SINGULAR
    '''
    J = analytic_jacobian(currQ, e)
    try:
        inv = np.linalg.inv(J)
        return 1
    except:
        return 0
    

def invkin(tolerance:int, maxiter: int, currQ, desiredP, e : rtb.Robot.ets, jointlimits : list, jacobian_method, plot_traj): #uses a constant Jacobian.
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

    ret = None

    for i in range(maxiter):
        currP = fkin(currQ, e)
        errorP = desiredP - currP
        error = np.linalg.norm(errorP)

        trajectory.append(currQ.copy())

        #print(J,"\n")
        if error < tolerance:
            ret =  1, currP, i, jac_updates
            break

        if jacobian_method==3:
            J=analytic_jacobian(currQ, e)
            jac_updates+=1;
        
        if jacobian_method==2:
            J = centraldiff_jacobian(currQ, e)
            jac_updates+=1;

        corrQ = np.linalg.pinv(J) @ errorP
        
        currQ += alpha * corrQ

        current_q = list(currQ.copy())
        '''
        for i in range(len(current_q)):
            if (current_q[i] < jointlimits[i][0]) or (current_q[i] > jointlimits[i][1]):

                ret =  0, currP, i, jac_updates
                break
        '''



    if plot_traj: #animate trajectory, very fast
        e.plot(np.array(trajectory), block=False)
        plt.close('all')
        #slider_robot_trajectory(np.array(trajectory))

    if not ret:
        ret =  0, currP, i, jac_updates

    return ret

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


def calculate_error(camera: mvtb.CentralCamera, tolerance, maxiter, resolution, e, jointlimits: list, jointlimits_full: list, desiredP, jacobian_method, plot_traj, plot_error):

    joint_ranges = [np.linspace(low, high, resolution) for (low, high) in jointlimits]

    grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)

    # preallocate error arrays (same shape as grid, but without dof)
    permuts_over_linspaces_shape = Q_grid.shape[:-1]

    sum_error=0;

    i=0;

    cartesian_points = []
    cartesian_error_values = []

    joints_near_goal = None

    for idx in np.ndindex(permuts_over_linspaces_shape): #iter over every permut over linspace of q0...qn 
                '''
                This for loop iterates over every pair/permutation in the linear spaces of our parameters,
                and returns the resulting point. 
                '''
                Q = Q_grid[idx].copy()
                cartesian_initial_x = fkin(Q, e)[0]
                cartesian_initial_y = fkin(Q, e)[1]
                cartesian_initial_z = fkin(Q, e)[2]
                cartesian_pos = np.array([cartesian_initial_x, cartesian_initial_y, cartesian_initial_z])
                cartesian_points.append(cartesian_pos)

                if camera:
                    pass # TODO add camera ughhh we need to add the camera eventually but i keep getting sidetracked with other simulations  #success, resultP, iterations, jac_updates = vs_invkin(camera, tolerance=tolerance, maxiter=maxiter, currQ=Q, desiredP=desiredP, e=e, jointlimits=jointlimits, mesh=mesh, jacobian_method=jacobian_method, simplex_mode=simplex_mode)
                else:
                    success, resultP, iterations, jac_updates = invkin(tolerance=tolerance, maxiter=maxiter, currQ=Q, desiredP=desiredP, e=e, jointlimits=jointlimits_full, jacobian_method=jacobian_method, plot_traj=0)
                error=np.linalg.norm(desiredP - resultP) 
                sum_error+=error

                if iterations <=2 and success==1:
                    success = 0.5
                    joints_near_goal = Q

                cartesian_error_values.append(success)

                print("i:", i, "init Q: ", Q_grid[idx], "fin Q:", Q, "result P:", resultP, "error:", error, "jac updates:", jac_updates)
                print("\n")
                i+=1;
    
    print("Sum of error: ", sum_error)
    print("Joints near goal position:", joints_near_goal)

    cartesian_points = np.array(cartesian_points)
    cartesian_error_values = np.array(cartesian_error_values)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    scatter = ax.scatter(cartesian_points[:, 0],  # X
                         cartesian_points[:, 1],  # Y
                         cartesian_points[:, 2],  # Z
                         c=cartesian_error_values,  # Color
                         cmap='plasma',
                         s=20)

    fig.colorbar(scatter, ax=ax, label='0=FAIL, .5= <=2 iterations, 1=SUCCESS')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Cartesian Error vs Initial Joint Configuration')
    plt.tight_layout()
    plt.show()

def find_singularities(resolution, e, jointlimits: list):

    joint_ranges = [np.linspace(low, high, resolution) for (low, high) in jointlimits]

    grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)

    # preallocate error arrays (same shape as grid, but without dof)
    permuts_over_linspaces_shape = Q_grid.shape[:-1]

    sum_error=0;

    i=0;

    cartesian_points = []
    cartesian_error_values = []

    joints_near_goal = None

    for idx in np.ndindex(permuts_over_linspaces_shape): #iter over every permut over linspace of q0...qn 
                '''
                This for loop iterates over every pair/permutation in the linear spaces of our parameters,
                and returns the resulting point. 
                '''
                Q = Q_grid[idx].copy()
                cartesian_initial_x = fkin(Q, e)[0]
                cartesian_initial_y = fkin(Q, e)[1]
                cartesian_initial_z = fkin(Q, e)[2]
                cartesian_pos = np.array([cartesian_initial_x, cartesian_initial_y, cartesian_initial_z])
                cartesian_points.append(cartesian_pos)

                singular = is_close_to_singular(Q, e)
            
                cartesian_error_values.append(singular)

        
                i+=1;
    

    cartesian_points = np.array(cartesian_points)
    cartesian_error_values = np.array(cartesian_error_values)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    scatter = ax.scatter(cartesian_points[:, 0],  # X
                         cartesian_points[:, 1],  # Y
                         cartesian_points[:, 2],  # Z
                         c=cartesian_error_values,  # Color
                         cmap='plasma',
                         s=20)

    fig.colorbar(scatter, ax=ax, label='0=singular, 1=not singular')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Singular? vs Initial Joint Configuration')
    plt.tight_layout()
    plt.show()


def main():
    l0=1;l1=1;l2=1;

    ets1dof = rtb.ET.Rz() * rtb.ET.tx(l0)
    joint_limits1dof = [(-np.pi/2, np.pi/2)]
    joint_limits1dof_full = [(-2*np.pi, 2*np.pi)]
    dof1 = ets1dof, joint_limits1dof, joint_limits1dof_full

    ets2dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rz() * rtb.ET.tx(l1) 
    joint_limits2dof = [(0, np.pi), (-np.pi, np.pi)]  # example for 2 DOF
    joint_limits2dof_full = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi)]
    dof2 = ets2dof, joint_limits2dof, joint_limits2dof_full

    ets3dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rx() * rtb.ET.tz(l1) * rtb.ET.Rx() * rtb.ET.tz(l2)
    joint_limits3dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits3dof_full = [(-2*np.pi/2, 2*np.pi/2), (-2*np.pi/2, 2*np.pi/2) , (-2*np.pi/2, 2*np.pi/2)]
    dof3 = ets3dof, joint_limits3dof, joint_limits3dof_full

    ets_dylan = rtb.ET.Rz() * rtb.ET.Ry(np.pi/2) * rtb.ET.Rz(np.pi) * rtb.ET.Ry() * rtb.ET.tz(0.55) * rtb.ET.Ry() * rtb.ET.tz(0.30)
    #joint_limits_dylan = [(0, np.pi/2), (0, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_dylan = [(-np.pi, np.pi), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_full_dylan = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi) , (-2*np.pi, 2*np.pi)]
    dofdylan = ets_dylan, joint_limits_dylan, joint_limits_full_dylan

    puma = Puma560()
    ets_puma = (puma.ets())  # shows ETS version
    joint_limits_puma= [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_puma_full = [(-2*np.pi/2, 2*np.pi/2), (-2*np.pi/2, 2*np.pi/2) , (-2*np.pi/2, 2*np.pi/2),(-2*np.pi/2, 2*np.pi/2), (-2*np.pi/2, 2*np.pi/2) , (-2*np.pi/2, 2*np.pi/2)]

    ets, joint_limits, joint_limits_full  = dofdylan

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
    tolerance = 1e-1
    maxiter = 200
    resolution=10
    
    #PLOTTING PARAMS
    desiredP = np.array([0., 0.39, 0.69])
    currQ = np.array([np.pi/2,np.pi/4,np.pi/4])

    #### JACOBIAN METHODS ####
    # 1 central diff, 2 central diff every update, 3 analytic every update (best possible)
    jacobian_method=1
    ################################################################

    print(fkin(currQ, ets))
    ets.plot(currQ, block=True)

    #success, resultP, iterations, jac_updates = invkin(tolerance, maxiter*2, currQ, desiredP, ets, joint_limits, joint_limits_full jacobian_method, 1)
    #print("init Q: ", currQ, "fin Q:", currQ, "result P:", resultP, "jac updates:", jac_updates)

    calculate_error(camera, tolerance, maxiter, resolution, ets, joint_limits, joint_limits_full, desiredP, jacobian_method, 0, 1)
    print("Des:", desiredP)

    #find_singularities(resolution, ets, joint_limits)
   
if __name__ == '__main__':
    main()
  
