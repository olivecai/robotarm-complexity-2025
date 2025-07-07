'''
June 10
This program should tell us which root each point 'chooses'. In other words, it shows us the basins of attraction.

For convenience, it will be easier to give the program distinct roots.
For our 2 DOF arm:

## Setup ##
Desired position at 1,1,0
Create a linearspace of joint angles: joint_limits2dof = [(0, np.pi), (-np.pi, np.pi)] 
Let the resolution be slightly high... ~40
Run 200-300 iterations of Newton's Method for each starting position to get to the desired position
For each starting position, when we terminate the loop, evaluate if we are near a solution:
    First, are we near the cartesian solution?
        If so, then for each TrueSolution, if CurrentSolution-TrueSolution differs by a low TOL (1e-1), then we classify that solution as part of this basin of attraction.
    If we are not near the cartesian solution, then we are not in any basin of attraction... color the point black.

## Outcomes ##
One of three outcomes will occur:
1. SUCCESS, attracted to solution pi/2, -pi/2
2. SUCCESS, attracted to solution 0, pi/2
3. FAILURE.

Have each basin be designated its own color (red, blue) and failures be black.
'''
import numpy as np
from common_robot_calculations import *
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import plotly.graph_objects as go

solutions = np.array([0., np.pi/2, -np.pi/2]), np.array([0., 0., np.pi/2])

#solutions for 2dof (1,1,0):(np.array([np.pi/2, -np.pi/2]), np.array([0, np.pi/2]))

def invkin(tolerance:int, maxiter: int, currQ, desiredP, e : rtb.Robot.ets, jointlimits : list): #uses a constant Jacobian.
    '''
    Inverse kinematics for constant Jacobian, simplices update, or every update, and supports simplices updates.
    No camera involved yet. Returns resulting position when restol satisfied OR when n=MAXITER
    '''

    alpha = 1 #dampening factor
    J = centraldiff_jacobian(currQ, e)
    
    for i in range(maxiter):
        currP = fkin(currQ, e)
        errorP = desiredP - currP
        error = np.linalg.norm(errorP)

        if error < tolerance:
            break

        corrQ = np.linalg.pinv(J) @ errorP
        #currQ=currQ.copy() #comment to modify currQ directly, uncomment to create a copy each time !!!!!
        currQ += alpha * corrQ

        '''
        for i in range(len(currQ)): 
            if currQ[i] > jointlimits[i][1] or currQ[i] < jointlimits[i][0]:
              return 0, currP, i '''
        
    for j in range(len(solutions)): 
        if (np.abs(currQ - solutions[j]) < 1e-1).all(): #if within tolerance...
            return j+1, currP, i
        
    return 0, currP, i

def compare_diff(Q):
    lowest_difference = np.inf
    best_sol = None
    for i in range(len(solutions)):
        diff = np.linalg.norm(Q-solutions[i])
        if diff < lowest_difference:
            lowest_difference = diff
            best_sol = i
    return best_sol
        
        
def calculate_error(tolerance, maxiter, resolution, e, jointlimits: list, desiredP):

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
                basin, resultP, iterations= invkin(tolerance=tolerance, maxiter=maxiter, currQ=Q, desiredP=desiredP, e=e, jointlimits=jointlimits)
                error=np.linalg.norm(desiredP - resultP) 
                sum_error+=error

                permuts_over_linspaces[idx] = basin

                #print("i:", i, "init Q: ", Q_grid[idx], "fin Q:", Q, "result P:", resultP, "error:", error, "iterations:", iterations)
                i+=1;
    
    print("Sum of error: ", sum_error)

    permuts_over_linspaces=permuts_over_linspaces.flatten()
    # Flatten Q_grid to get all [q0, q1, ..., qN] configs
    Q_flat = Q_grid.reshape(-1, Q_grid.shape[-1])
    x, y, z = Q_flat[:, 0], Q_flat[:, 1], Q_flat[:,2]
    colors= permuts_over_linspaces

    print(x.shape)
    print(y.shape)
    print(z.shape)
    print(colors.shape)

    print("Generating Plot...")
    
    fig=plt.figure(figsize=(8,6))
    ax = fig.add_subplot((111), projection='3d')

    scatter = ax.scatter(
        x,
        y,
        z,
        c=colors,
        marker = 'o',
        cmap='plasma',
        s=20)

    fig.colorbar(scatter, ax=ax, label='0==FAIL, otw SUCCESS')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Basins of Convergence')
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
    joint_limits2dof_full = [(-np.pi, np.pi), (-np.pi, 2*np.pi)]
    dof2 = ets2dof, joint_limits2dof, joint_limits2dof_full

    ets3dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rx() * rtb.ET.tz(l1) * rtb.ET.Rx() * rtb.ET.tz(l2)
    joint_limits3dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits3dof_full = [(-2*np.pi/2, 2*np.pi/2), (-2*np.pi/2, 2*np.pi/2) , (-2*np.pi/2, 2*np.pi/2)]
    dof3 = ets3dof, joint_limits3dof, joint_limits3dof_full

    ets_dylan = rtb.ET.Rz() * rtb.ET.Ry(np.pi/2) * rtb.ET.Rz(np.pi) * rtb.ET.Ry() * rtb.ET.tz(1.) * rtb.ET.Ry() * rtb.ET.tz(1.)
    #joint_limits_dylan = [(0, np.pi/2), (0, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_dylan = [(-np.pi, np.pi), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_full_dylan = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi) , (-2*np.pi, 2*np.pi)]
    dofdylan = ets_dylan, joint_limits_dylan, joint_limits_full_dylan

    ets, joint_limits, joint_limits_full  = dofdylan

    robot = rtb.Robot(ets)

    ############  toggle buttons ########################################

    #MESH PARAMS
    tolerance = 1e-3
    maxiter = 100
    resolution=10

    #PLOTTING PARAMS
    desiredP = np.array([1,0,1])

    calculate_error(tolerance, maxiter, resolution, ets, joint_limits, desiredP)
   

if __name__ == '__main__':
    main()
  