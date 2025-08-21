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
from mpl_toolkits.mplot3d import Axes3D

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

def calculate_longest_edge_midpoint(simplex: np.ndarray):
    '''
    Returns the midpoint coordinates of the longest edge of the simplex.
    simplex: np.ndarray of shape (n_vertices, n_dimensions)
    '''
    n = simplex.shape[0]
    max_dist = 0.0
    longest_edge_midpoint = None
    for i in range(n):
        for j in range(i+1, n):
            dist = np.linalg.norm(simplex[i] - simplex[j])
            if dist > max_dist:
                longest_edge_midpoint = (simplex[i]+simplex[j])/2
                max_dist = dist
                longest_edge_vertex1 = simplex[i]
                longest_edge_vertex2 = simplex[j]
    return longest_edge_midpoint, longest_edge_vertex1, longest_edge_vertex2

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

def interpolate_F(point, simplex):
    vertex_distances = []
    vertex_Fvals=[]
    sum_vertex_distances=0
    for vertex in simplex:
        vertex_distance = np.linalg.norm(np.subtract(point, simplex))
        vertex_distances.append(vertex_distance)
        sum_vertex_distances+=vertex_distance

        vertex_Fvals.append(robot.fkin_eval(*vertex)) #EVALUATE THE JACOBIAN

    weights=np.reshape((np.array(vertex_distances)/sum_vertex_distances),(1,-1))


    print(f"weights: {weights}\nvertex_Fvals: {vertex_Fvals}")
    interpolated_F = sum(w * J for w, J in zip(weights.flatten(), vertex_Fvals))

    print(f"interpolated F: {interpolated_F}")

    return interpolated_F



def refine_further_condition(true_value, interpolated_value):
    '''
    used recursively.
    returns True if the mesh should be refined further.
    returns False if the mesh refinement should halt.

    multiple conditions are possible.
    '''
    tol = 1e-1
    if np.linalg.norm(np.subtract(true_value, interpolated_value)) > tol:
        print("REFINE TRUE")
        return True
    else:
        print("REFINE FALSE")
        return False

def sparse_sample(jointlimits: list, robot: dh.DenavitHartenbergAnalytic, n = 2):
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

def conform_to_neighbour(simplex, edges_bisected: list, midpoint_associated_to_edge_that_was_bisected: list):
    '''
    Returns the midpoint (evaluated as True as a bool) if one of the edges in the simplex needs to be conformed to its neighbour.
    Returns None (evaluated as False as a bool) if there is no edge that needs to be conformed.
    '''
    
    for i in range(len(edges_bisected)):
        for simplex_edge in simplex:
            if edges_bisected[i] == simplex_edge: #then, one of the edges in this simplex was bisected before, and we need to subdivide. return the midpoint.
                return midpoint_associated_to_edge_that_was_bisected[i], simplex_edge
    return None, None

def mesh_objective(mode, centroid, simplex):
    if mode == 1: #compare jacobians
        true_value = robot.J(*centroid)
        interpolated_value = interpolate_jacobian(centroid, simplex)
    if mode == 2: #compare function value
        true_value = robot.fkin_eval(*centroid)
        interpolated_value = interpolate_F(centroid, simplex)
    return true_value, interpolated_value


def brute_force_rivara_refine(points_mesh: np.ndarray, initial_nodes: np.ndarray, mode: int):
    '''
    August 21 2025
    Brute Force Rivera Refinement: 
    Iterate through every simplice, create the new point to be refined along the longest edge,
    Then rebuild the Delaunay Mesh, which now invariably has more simplices.
    Repeat this process a few times. See how the mesh changes.

    The mesh will never have dangling edges.
    '''
    nodes : list = initial_nodes.copy().tolist()
    mesh_complete = 0 
    while not mesh_complete:
        mesh_complete=1
        prev_nodes = nodes
            
        for simplex in points_mesh:
            centroid = calculate_centroid(simplex)
            true_value, interpolated_value = mesh_objective(mode, centroid, simplex)
            print(f"true:\n {true_value}")
            print(f"interpolated:\n {interpolated_value}")

            if refine_further_condition(true_value, interpolated_value):
                mesh_complete=0
                simplex_longest_edge_midpoint, longest_edge_vertex1, longest_edge_vertex2 = calculate_longest_edge_midpoint(simplex)
                nodes.append(simplex_longest_edge_midpoint)

        nodes = np.unique(np.array(nodes), axis=0)
        nodes=list(nodes)
        mesh = Delaunay(nodes)

        #plot_mesh(mesh, nodes)
        points_mesh = np.array(nodes)[mesh.simplices]
    
    plot_mesh(mesh, nodes)
    return mesh, nodes


def rivara_refine(points_mesh: np.ndarray, initial_nodes: np.ndarray):
    '''
    Aug 21 2025

    If we need to further refine the mesh, we should look at the longest edge and create another node.
    For tetrahedrons: multidim rivara algorithm
    - find the longest edge 
    - find the midpoint of that longest edge
    - by definition of tetrahedron, there are two other points in the shape:
    - remove parent simplex,
    - and create two new child simplices from the new midpoint and the two other simplices in the shape 
    '''
    nodes = np.copy(initial_nodes)
    simplices_to_check_for_refinement = np.copy(points_mesh).tolist() #first in , first out. index 0 is the start of the list. append things to the back.

    edges_bisected = [] #in the rivara refinement, 
    # we need to keep track of which edges have been bisected,
    # so that we retain conformity in the triangles.

    while simplices_to_check_for_refinement:
        simplex = simplices_to_check_for_refinement[0] #check the first item
        centroid = calculate_centroid(simplex)
        true_value = robot.J(*centroid)
        interpolated_value = interpolate_jacobian(centroid, simplex)
        print(f"true:\n {true_value}")
        print(f"interpolated:\n {interpolated_value}")
        midpoint_of_neighbouring_edge, bisected_neighbouring_edge = conform_to_neighbour(simplex, edges_bisected)
        if midpoint_of_neighbouring_edge: 
            '''
            Aug 21 2025

            It would be smart if we could get each child and recurse and patch up the dangling noncomfortities as we go, but if it doesn't work it will be a nightmare.
            Idea: can we brute force the mesh by repeatedly finding children and then repairing the Delaunay mesh after every iteration? 
            This way we avoid recursion and we guarenteed do not have dangling pointers. 
            We can get the same result, actually, for the mesh, i
            '''
            # this indicates that a neighbouring simplex was bisected 
            # --> a nonconforming edge was created. 
            # thus we need to make two children out of the current simplex.
            simplices_to_check_for_refinement.pop(0)
            # TODO: get the children
            i+=1
            continue
        if refine_further_condition(true_value, interpolated_value):
            '''
            If we need to refine further then we need to get rid of the parent simplex.
            '''
            # TODO: finish this section
            simplex_longest_edge_midpoint, longest_edge_vertex1, longest_edge_vertex2 = calculate_longest_edge_midpoint(simplex)
        
        i+=1;

def plot_fkin(mesh, mesh_points, world_point_dimension=1):
    """
    Plot forward kinematics over a 2DOF mesh. 
    Z-axis = chosen world dimension.
    """
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111, projection='3d')
    mesh_points = np.array(mesh_points)

    # Evaluate FK for all mesh points
    world_points = np.array([robot.fkin_eval(*mesh_point) for mesh_point in mesh_points])
    world_points_specific = (np.array(world_points[:, world_point_dimension]).flatten())
    print("worldpoints\n", world_points)

    ax.scatter(
        mesh_points[:,0], 
        mesh_points[:,1],
        world_points_specific,
        color='blue'
    )

    for simplex in mesh.simplices:
        


        print(simplex)
        for i in range(3):
            p1 = mesh_points[simplex[i]]
            p2 = mesh_points[simplex[(i+1)%3]]
            print(p1)
            print(p2)
            p1_world = robot.fkin_eval(*p1)[world_point_dimension]
            p2_world = robot.fkin_eval(*p2)[world_point_dimension]

            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1_world[0], p2_world[0]], color='gray')


    ax.set_title("Delaunay Mesh mapped through FK (2DOF)")
    ax.set_xlabel("Joint 1")
    ax.set_ylabel("Joint 2")
    ax.set_zlabel(f"World dimension {world_point_dimension}")
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
            
            rivara_refine(points_mesh, initial_nodes)

if __name__ == '__main__':
    jointlimits = [(-np.pi/2, np.pi/2)]*robot.dof
    initial_mesh, initial_nodes = sparse_sample(jointlimits, robot)
    points_mesh= initial_nodes[initial_mesh.simplices]
    print(initial_mesh.simplices)
    print(points_mesh)
    plot_mesh(initial_mesh, initial_nodes)

    # mode 1 is for jacobian, mode 2 is for its function value
    mode = 2
    mesh, nodes = brute_force_rivara_refine(points_mesh, initial_nodes, mode)
    plot_fkin(mesh, nodes, mode)
    

class Mesh():
    def __init__(self, jointlimits, meshpoints=None):
        '''
        either pass precomputed mesh, or if None then calculate mesh from scratch.
        '''
        if meshpoints == None:
                
            initial_mesh, initial_nodes = sparse_sample(jointlimits, robot)
            points_mesh= initial_nodes[initial_mesh.simplices]
            print(initial_mesh.simplices)
            print(points_mesh)
            plot_mesh(initial_mesh, initial_nodes)

            # mode 1 is for jacobian, mode 2 is for its function value
            mode = 2
            mesh, nodes = brute_force_rivara_refine(points_mesh, initial_nodes, mode)
            self.mesh = mesh
            self.meshpoints = np.array(nodes)
            
        else:
            self.mesh = Delaunay(meshpoints)
            self.meshpoints = np.array(meshpoints)

    def interpolate_jac(self, point):
        simplex_index = self.mesh.find_simplex(point)
        simplex_vertex_indices = self.mesh.simplices[simplex_index]
        simplex = self.meshpoints[simplex_vertex_indices]
        jacobian = interpolate_jacobian(point, simplex)
        return jacobian
        
    def interpolate_f(self, point):
        simplex_index = self.mesh.find_simplex(point)
        simplex_vertex_indices = self.mesh.simplices[simplex_index]
        simplex = self.meshpoints[simplex_vertex_indices]
        f = interpolate_F(point, simplex)
        return f