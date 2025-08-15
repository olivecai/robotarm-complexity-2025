'''
August 15 2025

Can we make an online mesh for the robot?

Have a desired position in a well conditioned joint pose.

Generate random joint positions as our to-be-start points.

For every generated start point, perform Newton Policy and keep track of where the Jacobian update was needed by pushing it to a list.
During each Newton Policy, it's possible two trajectories will need to update near the same Jacobian, so for every Jacobian update, first verify that this point (within some tolerance) is not already in the mesh.

Should we make a mesh of simplices, or should we define 'hotspot' centers? For now let's see how the mesh of simplices looks.

What we will need:
- A list of points to send to scipy Delaunay in the end.
- The lipschitz constant of f'

'''

import numpy as np
import sympy as sp
import denavit_hartenberg as dh
import matplotlib.pyplot as plt

from scipy.spatial import Delaunay


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
robot = dof3

cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
vs.dh_robot.alpha = 1.

kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))

def generate_mesh_points(vs, desP, jointranges):
    '''
    Generate a series of random initial points to start the trajectory from. Consider the beginning Jacobian as a mesh point.
    '''
    mesh_points=[] #objective is to fill this list up
    num_init_points = 100

    random_joint_configs = [
        np.array([np.random.uniform(low, high) for (low, high) in jointranges])
        for _ in range(num_init_points)
    ]

    desPP = vs.projected_world_point(desP)

    for initQ in random_joint_configs:
        jac_policy(vs, initQ, desPP, mesh_points, jointranges)

    return mesh_points


def jacobian_update(curr_jacQ, meshpoints: list, jointranges):
    '''
    Aug 15 2025

    This function is called in jac_policy whenever we update the Jacobian.

    Update meshpoints by adding the current Jacobian Q if there does not already exist a Jacobian Q very similar to 
    '''
    for i in range(len(curr_jacQ)):
        if curr_jacQ[i] < jointranges[i][0]:
            return
        if curr_jacQ[i] > jointranges[i][1]:
            return

    print(f"Recv {curr_jacQ} as a possible meshpoint. Evaluating...")
    res_tol = 0.3
    for point in meshpoints:
        residual = np.subtract(curr_jacQ, point) 
        if ((np.array(np.abs(residual))) <= res_tol).all(): #if all entries of residual are less than tolerance, the Q are very similar. Else, different, so add that Q to the active meshpoints list.
            print(f"Rejected potential meshpoint {curr_jacQ}, due to existing mesh point {point}")
            return
    print(f"Adding {curr_jacQ} to the meshpoints list.")
    meshpoints.append(curr_jacQ.tolist())
    return

def jac_policy(robot: dh.DenavitHartenberg_Cameras_Analytic, initQ, desPP, meshpoints: list, jointranges):
    '''
    Aug 15 2025

    This function was born in measureJacobianError.py on August 11.
    It is modified here so that we can create our mesh of linear regions.
    '''

    currQ = initQ
    tolerance = 1e-3
    maxiter=50

    traj = [currQ]
    errors =[]
    jac_update_iters =[]


    J = robot.central_differences_pp(currQ, desPP)  
    
    delQ = [0.]*robot.dh_robot.dof


    robot.calc_lipschitz(4.0) #calculate the lipschitz constant

    jac_count=1

    epsilon = 1.0 #the amount of error permitted from the amount of distance moved from where we did the jacobian initial estimation 


    for i in range(maxiter):
        print("###################### i:", i)
        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current

        if np.linalg.norm(currError) <= tolerance:
            break

        Jinv = np.linalg.pinv(J)

        print("norm of Jinv:", np.linalg.norm(Jinv))

        corrQ = (Jinv @ currError).flatten()
        print(f"currQ: {currQ}\ncorrQ: {corrQ}\ncurrError: {currError}")
        
        delQ+=corrQ
        print("DELQ:", delQ)
        jacobian_approximated_error = robot.lipschitz / 2 * np.linalg.norm(delQ)**2
        
        print("jac approximated err:", jacobian_approximated_error)
    
        # Add the jacobian policy below!
        numerator = np.sqrt(2 * epsilon / robot.lipschitz)
        denominator = np.linalg.norm(delQ)

        lowerboundstep, updatethreshold, upperboundstep = (0.1, 0.2,  1)

        #when alpha is greater than 1, we tend to oscillate. But the step cannot be so small that it is neglible.
        step = min(upperboundstep, numerator/denominator)


        if step < lowerboundstep:
            step = lowerboundstep

        print("STEP:", step)

        if step < updatethreshold:
            delQ=[0.]*robot.dh_robot.dof
            J = robot.central_differences_pp(currQ, desPP)  
            curr_jacQ = currQ
            jac_count+=1
            jac_update_iters.append(i)
            
            print("Step too small; JACOBIAN UPDATE")
           

            jacobian_update(curr_jacQ, meshpoints, jointranges)
            

    
        currQ = currQ - step*corrQ
        traj.append(currQ)
        errors.append(np.linalg.norm(currError))

    print("Total jacobians used:", jac_count)

    traj = np.array(traj)
    if 0: 
        robot.dh_robot.rtb_robot.plot(traj, block=False)
        robot.dh_robot.plot(currQ)

    errors=np.array(errors)
    # fig, ax = plt.subplots()
    # ax.plot(range(len(errors)), errors, '-o', markersize=2, label="Error")

    # # Mark Jacobian update iterations with big red dots
    # ax.scatter(jac_update_iters, errors[jac_update_iters], 
    #         s=50, c='blue', zorder=3, label="Jacobian Update")

    # ax.set_xlabel("Iteration")
    # ax.set_ylabel("Error Norm")
    # ax.set_title("Error vs Iteration (Jacobian updates marked)")
    # ax.legend()
    # ax.grid(True)

    # plt.show()

    return

desQ = [1.5] * robot.dof
desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
jointranges = [(0, np.pi)] * vs.dh_robot.dof
mesh_points = generate_mesh_points(vs, desP, jointranges)
mesh = Delaunay(mesh_points)

print("mesh points list:", mesh_points)

#given the mesh, use it to perform Newton updates: should we assign each simplex its own Jacobian?




if vs.dh_robot.dof == 2:
    mesh_points = np.array(mesh_points)
    plt.figure(figsize=(8,8))
    plt.triplot(mesh_points[:,0], mesh_points[:,1], mesh.simplices, color='gray')
    plt.plot(mesh_points[:,0], mesh_points[:,1], 'o', color='blue')
    plt.title("Delaunay Mesh (2D)")
    plt.xlabel("Joint 1")
    plt.ylabel("Joint 2")
    plt.grid(True)
    plt.show()
else:
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111, projection='3d')
    mesh_points = np.array(mesh_points)
    ax.scatter(mesh_points[:,0], mesh_points[:,1], mesh_points[:,2], color='blue')

    for simplex in mesh.simplices:
        for i in range(3):
            p1 = mesh_points[simplex[i]]
            p2 = mesh_points[simplex[(i+1)%3]]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='gray')

    ax.set_title("Delaunay Mesh (3D)")
    ax.set_xlabel("Joint 1")
    ax.set_ylabel("Joint 2")
    ax.set_zlabel("Joint 3")
    plt.show()