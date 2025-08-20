'''
August 20 2025

The incentive in creating a mesh of the forward kinematics function:
- PROVE that the mesh is actually more simple than we think.
- INTERPOLATE mesh nodes so that we can precompute jacobians and then perform the inverse kinematics.

We will first focus on the mesh for the forward kinematics function.

IF most of the complexity from the visual servoing problem comes from the robot, then we should be able to use the forward kinematics jacobians to solve most simple constraints.

Though, of course, we should be able to generate a mesh for a number of different problems.

We can also generate the mesh for different constraints and see how the mesh changes.
'''
import denavit_hartenberg as dh
import sympy as sp
import numpy as np
import numpy as np
import math
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go
from scipy.spatial import Delaunay

from common_robot_calculations import *


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

initQ = [1.5] * robot.dof
desQ = [2.0] * robot.dof

desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
print("desired world point:", desP)
print("initial world point:", vs.dh_robot.fkin_eval(*initQ).flatten().tolist())


### HELPER FUNCTIONS ###
def calculate_centroid(simplex: np.ndarray):
    '''
    calculate the centroid of the simplex.
    given JOINT ANGLES return the JOINT ANGLES.
    do not take nor pass any other information about the simplices.
    this makes the methods more modular.
    '''
    d = simplex.shape[0]
    n = simplex.shape[1]
    centroid = np.zeros(n)
    for i in range(n):
        for j in range(d):
            centroid[i] += simplex[j][i]
        centroid[i] /= d
    return centroid 

def calculate_longest_edge(simplex: np.ndarray):
    '''
    Returns the length of the longest edge of the simplex.
    simplex: np.ndarray of shape (n_vertices, n_dimensions)
    '''
    n = simplex.shape[0]
    max_dist = 0.0
    for i in range(n):
        for j in range(i+1, n):
            dist = np.linalg.norm(simplex[i] - simplex[j])
            if dist > max_dist:
                max_dist = dist
    return max_dist

def interpolate_jacobian(point, simplex):
    '''
    assume point is in the simplex.
    total distance between point and each vertex = 0
    vertex distances = []
    for each vertex in the simplex, 
        append the distance between point and vertex i
        add that distance to the total distance

    perform the element wise division so that the total distances sum to 1:
    [distance i/ tot distance, distance i+1/tot distance, ... ] (either 3 or 4 vertices, depending on 2 or higher dof)

    then, the interpolated jacobian is equal to the sum of: each jacobian * element wise multiplication with the scalar weight calculated above from the vertices

    return the interpolated jacobian.

    NOTE that for the centroid, each point is obviously an equal distance from each vertex.
    '''
    vertex_distances = []
    vertex_jacobians=[]
    sum_vertex_distances=0
    for vertex in simplex:
        vertex_distance = np.linalg.norm(np.subtract(point, simplex))
        vertex_distances.append(vertex_distance)
        sum_vertex_distances+=vertex_distance

        vertex_jacobians.append(robot.J(*vertex)) #EVALUATE THE JACOBIAN

    weights=np.reshape((np.array(vertex_distances)/sum_vertex_distances),(1,-1))


    print(f"weights: {weights}\nvertex_jacobians: {vertex_jacobians}")
    interpolated_jacobian = sum(w * J for w, J in zip(weights.flatten(), vertex_jacobians))

    print(f"interpolated jac: {interpolated_jacobian}")

    return interpolated_jacobian


def refine_further_condition(true_value, interpolated_value):
    '''
    used recursively.
    returns True if the mesh should be refined further.
    returns False if the mesh refinement should halt.

    multiple conditions are possible.
    we will consider Jacobian variation first.
    '''
    tol = 1e-1
    if np.linalg.norm(np.subtract(true_value, interpolated_value)) > tol:
        print("REFINE TRUE")
        return True
    else:
        return False

def sparse_sample(jointlimits: list, robot: dh.DenavitHartenbergAnalytic, n = 3):
    '''
    create a sparse grid over the space 
    create a delaunay mesh of the sparse grid points.
    '''
    joint_ranges = [np.linspace(low, high, n) for (low, high) in jointlimits]
    grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)
    # preallocate error arrays (same shape as grid, but without dof)
    permuts_over_linspaces_shape = Q_grid.shape[:-1]

    initial_nodes=[]

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
                J = robot.J(*Q)
                print(f"{i}: {Q}\n, {J}")

                #these are the vertices. initialize the delaunay points on these starting vertices.
                initial_nodes.append(Q)
            
                i+=1;
    
    initial_mesh = Delaunay(initial_nodes)

    return initial_mesh, np.array(initial_nodes)


def plot_mesh(mesh, mesh_points):
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

def refine(points_mesh, initial_nodes):
    nodes = np.copy(initial_nodes)
    for simplex in points_mesh:
        centroid = calculate_centroid(simplex)
        true_value = robot.J(*centroid)
        interpolated_value = interpolate_jacobian(centroid, simplex)
        print(f"true:\n {true_value}")
        print(f"interpolated:\n {interpolated_value}")
        if refine_further_condition(true_value, interpolated_value):
            simplex_longest_edge = calculate_longest_edge(simplex)
        

     
jointlimits = [(-np.pi/2, np.pi/2)]*robot.dof
initial_mesh, initial_nodes = sparse_sample(jointlimits, robot)
print(initial_mesh.simplices)
print(initial_nodes[initial_mesh.simplices])
plot_mesh(initial_mesh, initial_nodes)
points_mesh= initial_nodes[initial_mesh.simplices]

refine(points_mesh, initial_nodes)

