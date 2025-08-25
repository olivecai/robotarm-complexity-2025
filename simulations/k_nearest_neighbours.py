'''
August 22

sample k random points, store the jacobian for each point.


'''

import numpy as np
import sympy as sp
import denavit_hartenberg as dh
import matplotlib.pyplot as plt

P = dh.DHSympyParams() #this lowkey has to be global, sorry
jntspace, cartspace, taskspace = P.get_params()
t0, t1, t2, t3, t4, t5, t6, t7, t8, t9 = jntspace
x, y, z = cartspace
u, v = taskspace

dof2_params = [
                [t0, 0, 1, 0], 
                [t1, 0, 1, 0]
                ]

dylan_dof3_params=[
                [ t0, sp.pi/2, 0 , 0 ],
                [ t1,  0  ,  0.55, 0 ],
                [ t2,  0  ,  0.3, 0 ]
                ]


kinova_dof7_params = [
    [t0,      sp.pi,   0.0,   0.0],
    [t1,      sp.pi/2, 0.0, -(0.1564 + 0.1284)],
    [t2 +sp.pi, sp.pi/2, 0.0, -(0.0054 + 0.0064)],
    [t3 +sp.pi, sp.pi/2, 0.0, -(0.2104 + 0.2104)],
    [t4 +sp.pi, sp.pi/2, 0.0, -(0.0064 + 0.0064)],
    [t5 +sp.pi, sp.pi/2, 0.0, -(0.2084 + 0.1059)],
    [t6 +sp.pi, sp.pi/2, 0.0, 0.0],
    [t7 +sp.pi,    sp.pi, 0.0, -(0.1059 + 0.0615)],
]

dof2 = dh.DenavitHartenbergAnalytic(dof2_params, P)
dof3 = dh.DenavitHartenbergAnalytic(dylan_dof3_params, P)
kinova = dh.DenavitHartenbergAnalytic(kinova_dof7_params, P)
robot = dof2

print("ROBOT:\n",robot.J_analytic)

cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
vs.dh_robot.alpha = 1.

kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))
kinova_end = np.deg2rad(np.array([25.336059570312672, 50.57940673828129, -179.4915313720703, -90.7, 30.06742369383573531, -57.420898437500036, 30.88030242919922, 0.5]))

# initialize init Q and de
# initialize init Q and des P

initQ = [1.0] * robot.dof
desQ = [2.0] * robot.dof

desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
print("desired world point:", desP)
print("initial world point:", vs.dh_robot.fkin_eval(*initQ).flatten().tolist())

def k_sample_pnts(vs: dh.DenavitHartenberg_Cameras_Analytic, desP, jointranges):
    '''
    Generate a series of k random initial points and get their jacobians
    Parallel lists
    '''
    random=0
    desPP = vs.projected_world_point(desP)
    if random: 
        num_init_points = 400

        joint_configs = [
            np.array([np.random.uniform(low, high) for (low, high) in jointranges])
            for _ in range(num_init_points)
        ]

        stored_jacobians = []


        for Q in joint_configs:
            stored_jacobians.append(vs.central_differences_pp(Q, desPP))
    else:
        n = 4
        joint_ranges = [np.linspace(low, high, n) for (low, high) in jointlimits]
        grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

        # stack all grids to create a dof x n x n x ... array
        Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)
        # preallocate error arrays (same shape as grid, but without dof)
        permuts_over_linspaces_shape = Q_grid.shape[:-1]

        joint_configs=[]
        stored_jacobians = []

        i=0;
        for idx in np.ndindex(permuts_over_linspaces_shape): #iter over every permut over linspace of q0...qn 
                '''
                This for loop iterates over every pair/permutation in the linear spaces of our parameters:
                Example: if, for our 2DOF arm, joint_limits = [(-pi/2, pi/2), (-pi/2, pi/2)] and sparse_step=3, 
                iter 0 : idx = (0, 0) Q = [-1.57079633 -1.57079633] meshpoint = [-1.57079633 -1.57079633 -1.        ]
                iter 1 : idx = (0, 1) Q = [-1.57079633  0.        ] meshpoint = [-1.57079633e+00  0.00000000e+00  1.22464680e-16]
                iter 2 : idx = (0, 2) Q = [-1.57079633  1.57079633] meshpoint = [-1.57079633  1.57079633  1.        ]
                iter 3 : idx = (1, 0) Q = [ 0.         -1.57079633] meshpoint = [ 0.         -1.57079633  1.        ]
                iter 4 : idx = (1, 1) Q = [0. 0.] meshpoint = [0. 0. 2.]
                iter 5 : idx = (1, 2) Q = [0.         1.57079633] meshpoint = [0.         1.57079633 1.        ]
                iter 6 : idx = (2, 0) Q = [ 1.57079633 -1.57079633] meshpoint = [ 1.57079633 -1.57079633  1.        ]
                iter 7 : idx = (2, 1) Q = [1.57079633 0.        ] meshpoint = [1.57079633e+00 0.00000000e+00 1.22464680e-16]
                iter 8 : idx = (2, 2) Q = [1.57079633 1.57079633] meshpoint = [ 1.57079633  1.57079633 -1.        ]

                number of iterations = sparse_step raised to the DOF (sparse_step ** DOF)
                meshpoint simply is Q, but with an extra entry, being the position coord. 
                    Here, X=0, so we index the real world position vector (solved from forward kinematics) at index 0, getting the x coordinate.
                '''
                Q = Q_grid[idx] 
                J = vs.central_differences_pp(Q, desPP)
                print(f"{i}: {Q}\n, {J}")

                stored_jacobians.append(J)

                #these are the vertices. initialize the delaunay points on these starting vertices.
                joint_configs.append(Q)
            
                i+=1;         
         

    return joint_configs, stored_jacobians


def k_nearest_neighbour_jac(Q, desPP, storedjoints, storedjacobians, k_to_take = 100):
    '''
    interpolated jacobian
    '''
     #sample k of the closest neighbours

    distances = [np.linalg.norm(np.subtract(Q, storedQ)) for storedQ in storedjoints]
    # Get indices of the k smallest distances
    nearest_indices = np.argsort(distances)[:k_to_take]
    nearest_distances = np.array(distances)[nearest_indices]
    nearest_jacobians = [storedjacobians[i] for i in nearest_indices]
    nearest_Q = [stored_joints[i] for i in nearest_indices]

    print("NEAREST Q:", nearest_Q)

    # sum_distances = np.sum(nearest_distances)
    weights = [i**-1 for i in nearest_distances]
    sum_distances = np.sum(weights)
    weights = weights/sum_distances
    #weights = nearest_distances / sum_distances

    print(f"weights: {weights}\njacobians: {nearest_jacobians}")
    interpolated_jacobian = sum(w * J for w, J in zip(weights, nearest_jacobians))

    print(f"interpolated jac: {interpolated_jacobian}")
    actual_jac = vs.central_differences_pp(Q, desPP)
    print(f"actual jacobian: {actual_jac}")

    print(f"difference between actual and interpolated jacobian:\n {np.subtract(interpolated_jacobian, actual_jac)}")

    return interpolated_jacobian

def trial(robot: dh.DenavitHartenberg_Cameras_Analytic, stored_joints : list, stored_jacobians: list):
    '''
    Complete a trajectory while using an interpolated jacobian 
    '''
    ret = 0 #failure

    currQ = initQ
    tolerance = 1e-2
    maxiter=50

    traj = [currQ] #trajectory to plot later
    errors =[]
    jac_update_iters =[]

    desPP = robot.projected_world_point(desP)

    # robot.calc_lipschitz(2.0) #calculate the lipschitz constant

    jac_count=1

    epsilon = 1.0 #the amount of error permitted from the amount of distance moved from where we did the jacobian initial estimation 

    n_consecutive_small_steps=0

    
    step= 0.5

    #but maybe we shoudl only choose SOME not all

    for i in range(maxiter):

        J = k_nearest_neighbour_jac(currQ, desPP, stored_joints, stored_jacobians)
        print("J:\n", J)

        print("###################### i:", i)
        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current

        if np.linalg.norm(currError) <= tolerance:
            ret = 1
            break

        Jinv = np.linalg.pinv(J)

        print("norm of Jinv:", np.linalg.norm(Jinv))

        corrQ = (Jinv @ currError).flatten()
        print(f"currQ: {currQ}\ncorrQ: {corrQ}\ncurrError: {currError}")
        
        currQ = currQ - step*corrQ
        traj.append(currQ)
        errors.append(np.linalg.norm(currError))

    traj = np.array(traj)
    if 1: 
        robot.dh_robot.rtb_robot.plot(traj, block=False)
        robot.dh_robot.plot(currQ)

    if 1:
        errors=np.array(errors)
        fig, ax = plt.subplots()
        ax.plot(range(len(errors)), errors, '-o', markersize=2, label="Error")

        # Mark Jacobian update iterations with big red dots
        ax.scatter(jac_update_iters, errors[jac_update_iters], 
                s=50, c='blue', zorder=3, label="Jacobian Update")

        ax.set_xlabel("Iteration")
        ax.set_ylabel("Error Norm")
        ax.set_title("Error vs Iteration (Jacobian updates marked)")
        ax.legend()
        ax.grid(True)

        plt.show()

    return ret

jointlimits = [(-np.pi, np.pi)]*robot.dof
success_vals = 0
trials_count = 3
for _ in range(trials_count):
    stored_joints, stored_jacobians = k_sample_pnts(vs, desP, jointlimits)
    print("STORED JOINTS:", stored_joints)
    print("STORED JACS:", stored_jacobians)
    success_vals += trial(vs, stored_joints, stored_jacobians)
print(f"success vals: {success_vals}")
success_percent = success_vals/trials_count
print("SUCCESS PERCENTAGE:", success_percent)
