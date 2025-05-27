'''
May 16 2025 

This program uses successive mesh refinement and recursion, 
recursively refining local areas of the mesh until the 
error of the centroid of our mesh shape < RESTOL, the residual tolerance. We use an ERobot, which is simply a robot made from an ETS from the rtb.

This program needs to know:
- the real position of the robot that is achieved using forward kinematics of specified parameter configurations
- what the camera sees as the position and the translation of the camera point percieved position obtained from our system

1-DOF: curve, lines
2-DOF: surface, triangle
3-DOF: volume, tetrahedral

Use scipy.spatial Delaunay to triangulate/tetrahedralate(?) in any dimension.

We use triangles for 2DOF, tetrahedrons for 3 DOF and higher.

Objects like triangles and tetrahedrons are built as the program progresses and are not updated if the mesh is reformed.
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

def calculate_centroid(points):
    '''
    calculates the centroid of any shape.

    points: tuple containing d np.arrays, each np.array is n-dim.
    for instance, 
    triangle in R2 : d=3, n=2, (x1+x2+x3 / 3, y2+y2+y3/3)
    triangle in Rn : d=3, n=n, ( xa1+xa2+xa3 / 3, xb1+xb2+xb3 /3, ... xn1+xn2+xn3/3)
    tetrahedral in R3 : d=4, n=3, (x1+x2+x3+x4 / 4, y2+y2+y3+y4/4,  z2+z2+z3+z4/4, w2+w2+w3+w4/4,)

    returns [ a. b. c. ... n. ] where a,b,c,..,n are all floats
    '''
    d= points.ddim
    n= points.ndim
    print(n)
    print(d)
    centroid = np.zeros(n)
    print(centroid)
    for i in range(n):
        for j in range(d):
            centroid[i] += points.structure[j][i]
        centroid[i] /= d
    return centroid

def forward_kin(q, ets: rtb.ETS):
    '''
    return the real world/actual position in R3 of the end effector based on the specified joint parameters, q (np array).
    '''
    T = ets.eval(q)       
    x=T[0][3];
    y=T[1][3];
    z=T[2][3];
    return np.array([x,y,z]);

def newshape(Shape, ith, adjustedcentroid):
    '''
    this function creates a new SHAPE and the last index is the new POINT

    adjustedcentroid is the new point which we would like to make a new shape with. 
    ith is the point of the parent shape which we exclude (and swap for adjustedcentroid). 
    creates and returns np.array, shape (ddim,).'''
    ret = []
    print(ret)
    j=0
    for i in range(Shape.ddim):
        print("THIS IS i:", i)
        if i!=ith:
            print(Shape.structure[i])
            ret.append(Shape.structure[i])
            j+=1;
        print(ret)
    ret.append(adjustedcentroid)
    return ret

class Triangle:
    #looks like triangles would only be used for 2DOF case? would look like Cole's diagrams with 2DOF params x-y and position on z axis
    def __init__(self, point1, point2, point3):
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3
        self.ddim = 3 #by definition of triangle 
        self.ndim = point1.shape[0]
        points=(self.point1, self.point2, self.point3)
        self.structure = points
        self.centroid = calculate_centroid(self)

class Tetrahedron:
    def __init__(self, point1, point2, point3, point4):
        self.point1 = point1
        self.point2 = point2
        self.point3 = point3
        self.point4 = point4
        self.ddim = 4 #by definition of tetrahedron
        self.ndim = point1.shape[0]
        points=(self.point1, self.point2, self.point3, self.point4)
        self.structure = points 
        self.centroid = calculate_centroid(self)

class DelaunayMesh:
    def __init__(self, restol: float, robot: rtb.ERobot, camera: CentralCamera, sparse_step: int, jointlimits=list ): #using robots defined by ETS
        self.restol = restol
        self.robot = robot
        self.q_count= robot.ets().n
        self.jointlimits = jointlimits #joint_limits = [(-np.pi, np.pi), (-np.pi/2, np.pi/2), (-1, 2)]  # example for 3 DOF
        self.camera = camera #for now we wont use the camera. TODO: camera unused
        self.iterations=0 # incremented every time another point is plotted into our mesh
        self.nodes = [] #ordered list of the nodes we currently have in the mesh. for one item in the list we have: xa1, xa2, ..., xan, position in real world (the last index may be modified, TBD)
        self.plotnodes= [] #clone of .nodes, but without the world dimensions
        if self.q_count <= 2: #2DOF use triangles.
            self.shape_vertices_count=3 
        else:
            self.shape_vertices_count=4
        self.sparse_step = sparse_step #initial grid size: how many datapoints for EACH q range? this is essentially the step size in the linear space 

def recursive_add_triangles(mesh: DelaunayMesh, parent: Triangle):
    '''
    TRIANGLES
    update iterations and nodes for each point that is added to the mesh '''
    #calculate the centroid. compare centroid to the real point at the specified angles.
    centroid = parent.centroid 
    q = centroid[:mesh.q_count] #extract q, since the remaining elements in the array will be position coords
    posMesh = centroid[mesh.q_count:] #[x, y, z]
    posR = (forward_kin(q, mesh.robot.ets()))
    residual = np.linalg.norm(posR - posMesh) #calculate the residual
    if residual > mesh.restol: #then we should mesh again at the centroid and recurse on each child. for triangles, 3 children are created; tetrahedrons, 4.
        for i in range(parent.ddim): #the number of vertices correspond to the number of new shapes created internally.
            print("THIS IS ith:", i)
            centroid[mesh.q_count:] = posR #if the residual is larger than restol, we should refine the mesh at this local point.
            
            ith_newshape= newshape(parent, i, centroid) #newshape is generating three points for us here.
            mesh.iterations+=1 #bookkeeping
            mesh.nodes.append(ith_newshape[2]) #bookkeeping, last point is always the new point
            mesh.plotnodes.append(ith_newshape[2][:mesh.q_count])
            child=Triangle(ith_newshape[0], ith_newshape[1], ith_newshape[2])  
            return recursive_add_triangles(mesh, child)    
    else:
        return 
    
def recursive_add_tetrahedrons(mesh: DelaunayMesh, parent: Triangle):
    '''
    TETRAHEDRONS
    update iterations and nodes for each point that is added to the mesh '''
    #calculate the centroid. compare centroid to the real point at the specified angles.
    centroid = parent.centroid 
    q = centroid[:mesh.q_count] #extract q, since the remaining elements in the array may be position coords
    posMesh = centroid[mesh.q_count:] 
    posR = (forward_kin(q, mesh.robot.ets()))
    residual = np.linalg.norm(posR - posMesh) #using np linalg norm since we might want to expand later.

    if residual > mesh.restol: #then we should mesh again with the centroid q and pos as the real pos and recurse on each child. for triangles, 3 children are created; tetrahedrons, 4.
        for i in range(parent.ddim): #ie for i in range(parent.ddim)
            centroid[mesh.q_count:] = posR
            ith_newshape= newshape(parent, i, centroid)
            mesh.iterations+=1 #bookkeepgin
            mesh.nodes.append(ith_newshape[3]) #bookkeeping, last point is new point
            mesh.plotnodes.append(ith_newshape[3][:mesh.q_count])
            child=Tetrahedron(ith_newshape[0], ith_newshape[1], ith_newshape[2], ith_newshape[3])  
            return recursive_add_tetrahedrons(mesh, child)    
    else:
        return 
    
def create_sparsespace(Mesh: DelaunayMesh):
    '''
    this is where we initialize the mesh using a constant grid and also call our recursive functions.
    1. generate linspace for each dof
    2. create a meshgrid and plot our points.
    3. if tetrahedrons: for every four points make shape & call recursive_add_tetrahedrons
        OR triangles: for every three points, call recursive_add_triangles
    4. our recursive mesh helper fns add points into our list of points as needed. 
    5. by the end of the recursive steps, we should have a list with many different points in it.
    6. pass this list into scipy delaunay mesh.
    7. NOTE: as we recurse, we create the notion of triangles/tetrahedrals in order to find the centroid. 
        However, it is not for CERTAIN (as far as i currently know) that scipy will create mesh with the same triangles; 
        ie a better mesh may be available after obtaining all points. This is fine for now;
        We simply keep the notion of triangles in order to find the centroid.
    '''
    n = Mesh.sparse_step #resolution. NOTE: should be very low, maybe n=2
    # create linspaces and meshgrid
    joint_ranges = [np.linspace(low, high, n) for (low, high) in Mesh.jointlimits]
    grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)

    # preallocate error arrays (same shape as grid, but without dof)
    permuts_over_linspaces_shape = Q_grid.shape[:-1]
    permuts_over_linspaces = np.zeros(permuts_over_linspaces_shape)

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
                meshpoint = np.zeros(Q.shape[0] + 3) #add three, for the world dimensions.
                posR = forward_kin(q=Q, ets= Mesh.robot.ets())
                meshpoint[: Q.shape[0]] = Q
                meshpoint[Q.shape[0]:] = posR

                print("iter",i,": idx =",idx, "Q =",Q, "meshpoint =",meshpoint)
                Mesh.nodes.append(meshpoint)
                Mesh.plotnodes.append(Q)
                i+=1;
    
    '''
    At this point we have created our sparse grid and it is now time to iterate over that grid and perform recursion on non-overlapping triangles/tetrahedrons.
    '''

    #call recursive add triangle (or tetrahedron) for every collection of consecutive 3 (or 4) points:
    initmeshpoints=np.copy(Mesh.nodes) #copy over the initializing nodes so that we can iterate over them. the mesh.nodes list is going to be modified concrrently.
    i=0;
    if Mesh.shape_vertices_count == 3: #triangles
        while i+2 < (Mesh.sparse_step ** Mesh.q_count):
            triangle = Triangle(initmeshpoints[i], initmeshpoints[i+1], initmeshpoints[i+2])
            recursive_add_triangles(Mesh, triangle)
            i+=1;
    if Mesh.shape_vertices_count ==4: #tetrahedrons
        while i+3 < (Mesh.sparse_step ** Mesh.q_count):
            tetrahedron = Tetrahedron(initmeshpoints[i], initmeshpoints[i+1], initmeshpoints[i+2], initmeshpoints[i+3])
            recursive_add_tetrahedrons(Mesh, tetrahedron)
            i+=1;

    '''
    At this point we have finished recursion and mesh.nodes should be populated. Now we run Delaunay Meshing on the nodes. 
    When plotting, create a copy of the list of nodes we have and erase their last three dims to only see DOF.'''

def create_delaunaymesh_1DOF(Mesh: DelaunayMesh, mode: int):
    '''
    Plot Delaunay Mesh for 1 DOF arm.
    mode==1: display 'flattened' 1D graph
    mode==2: display 3D graph, with x as q0, y and z as x and y cartesian coord
    '''
    plotnodes=np.array(Mesh.plotnodes) #convert list of np arrays into a double nested np array
    nodes=np.array(Mesh.nodes)
    posnodes=np.array([np.delete(sub_arr, Mesh.q_count+2) for sub_arr in nodes])
    if mode==1:
        print("POSNODES:" , posnodes)
        dela = Delaunay(posnodes)  
        plt.figure()
        ax = plt.axes(projection='3d')
        print("dela.simplices count:", len(dela.simplices))
        for tr in dela.simplices:
            pts = dela.points[tr, :]
            ax.plot3D(pts[[0,1],0], pts[[0,1],1], pts[[0,1],2], color='g', lw='0.1')
            ax.plot3D(pts[[0,2],0], pts[[0,2],1], pts[[0,2],2], color='g', lw='0.1')
            ax.plot3D(pts[[0,3],0], pts[[0,3],1], pts[[0,3],2], color='g', lw='0.1')
            ax.plot3D(pts[[1,2],0], pts[[1,2],1], pts[[1,2],2], color='g', lw='0.1')
            ax.plot3D(pts[[1,3],0], pts[[1,3],1], pts[[1,3],2], color='g', lw='0.1')
            ax.plot3D(pts[[2,3],0], pts[[2,3],1], pts[[2,3],2], color='g', lw='0.1')

        ax.scatter(dela.points[:,0], dela.points[:,1], dela.points[:,2], color='b')
        ax.set_xlabel('q0')
        ax.set_ylabel('X coord')
        ax.set_zlabel('Y coord')
    if mode==2:
        print("PLOTNODES:", plotnodes)
        
        # Create a figure and an axes object
        fig, ax = plt.subplots(figsize=(8, 1))  # Adjust figure size as needed

        # Plot the points
        ax.plot(plotnodes, np.zeros_like(plotnodes), 'o', markersize=8) # 'o' for circle markers

        # Customize the plot
        ax.set_yticks([])  # Remove y-axis ticks and labels
        ax.set_xlabel("q0")
        ax.set_title("1DOF mesh ")

        # Remove spines (borders) for a cleaner look
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(False)

    plt.show()
    

def create_delaunaymesh_2DOF(Mesh: DelaunayMesh, mode: int):
    '''
    Create and plot a Delaunay Mesh for a 2DOF arm.
    mode==1: display 'flattened' 2D graph
    mode==2: display 3D graph with z-axis as X-pos
    mode==3: display 3D graph with z-axis as Y-pos
    mode==4: display 3D graph with z-axis as Z-pos
    '''

    print("Creating Delaunay Mesh...")
    
    #print("Collection of nodes to mesh over:",Mesh.plotnodes)

    plotnodes=np.array(Mesh.plotnodes) #convert list of np arrays into a double nested np array
    nodes=np.array(Mesh.nodes)

    if mode == 2: #show x pos
        a=1; b=2
    if mode == 3: #show y pos
        a=0; b=2
    if mode == 4: #show z pos
        a=0; b=1
    if not mode ==1:
        posnodes=np.array([np.delete(sub_arr, [Mesh.q_count+a, Mesh.q_count+b]) for sub_arr in nodes])
    
    if Mesh.shape_vertices_count == 3:
        if mode==1: #2D plot
            dmesh = Delaunay(plotnodes)
            plt.triplot(plotnodes[:,0], plotnodes[:,1], dmesh.simplices)
            plt.plot(plotnodes[:,0], plotnodes[:,1], 'o')
            plt.show()
        if mode >= 2: #use a 3D plot 
            dmesh = Delaunay(posnodes)  
            plt.figure()
            ax = plt.axes(projection='3d')
            plot_mesh_2DOF(ax, dmesh, mode)
            plt.show()

def plot_mesh_2DOF(ax, dela, mode):
    print("dela.simplices count:", len(dela.simplices))
    for tr in dela.simplices:
        pts = dela.points[tr, :]
        ax.plot3D(pts[[0,1],0], pts[[0,1],1], pts[[0,1],2], color='g', lw='0.1')
        ax.plot3D(pts[[0,2],0], pts[[0,2],1], pts[[0,2],2], color='g', lw='0.1')
        ax.plot3D(pts[[0,3],0], pts[[0,3],1], pts[[0,3],2], color='g', lw='0.1')
        ax.plot3D(pts[[1,2],0], pts[[1,2],1], pts[[1,2],2], color='g', lw='0.1')
        ax.plot3D(pts[[1,3],0], pts[[1,3],1], pts[[1,3],2], color='g', lw='0.1')
        ax.plot3D(pts[[2,3],0], pts[[2,3],1], pts[[2,3],2], color='g', lw='0.1')

    ax.scatter(dela.points[:,0], dela.points[:,1], dela.points[:,2], color='b')
    ax.set_xlabel('q0')
    ax.set_ylabel('q1')
    if mode ==2: 
        label='X'
    if mode ==3:
        label='Y'
    if mode == 4: 
        label=='Z'
    ax.set_zlabel(label+'coordinate')

def plot_mesh_3DOF(ax, dela):
    print("dela.simplices count:", len(dela.simplices))
    for tr in dela.simplices:
        pts = dela.points[tr, :]
        ax.plot3D(pts[[0,1],0], pts[[0,1],1], pts[[0,1],2], color='g', lw='0.1')
        ax.plot3D(pts[[0,2],0], pts[[0,2],1], pts[[0,2],2], color='g', lw='0.1')
        ax.plot3D(pts[[0,3],0], pts[[0,3],1], pts[[0,3],2], color='g', lw='0.1')
        ax.plot3D(pts[[1,2],0], pts[[1,2],1], pts[[1,2],2], color='g', lw='0.1')
        ax.plot3D(pts[[1,3],0], pts[[1,3],1], pts[[1,3],2], color='g', lw='0.1')
        ax.plot3D(pts[[2,3],0], pts[[2,3],1], pts[[2,3],2], color='g', lw='0.1')

    ax.scatter(dela.points[:,0], dela.points[:,1], dela.points[:,2], color='b')
    ax.set_xlabel('q0')
    ax.set_ylabel('q1')
    ax.set_zlabel('q2')

def create_delaunaymesh_3DOF(Mesh: DelaunayMesh):
    plotnodes=np.array(Mesh.plotnodes) #convert list of np arrays into a double nested np array
    nodes=np.array(Mesh.nodes)
    dmesh = Delaunay(plotnodes)
    plt.figure()
    ax = plt.axes(projection='3d')
    plot_mesh_3DOF(ax, dmesh)
    plt.show()
    
def main():
    l0=1;l1=1;l2=1;

    ets1dof = rtb.ET.Rz() * rtb.ET.tx(l0)
    joint_limits1dof = [(-np.pi, np.pi)]

    ets2dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rz() * rtb.ET.tx(l1) 
    joint_limits2dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2)]  # example for 2 DOF

    ets3dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rx() * rtb.ET.tz(l1) * rtb.ET.Rx() * rtb.ET.tz(l2)
    joint_limits3dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]  # example for 2 DOF

    joint_limits = joint_limits1dof
    ets=ets1dof 

    camera = CentralCamera()
    robot = rtb.Robot(ets)

    mesh = DelaunayMesh(1e-1, robot, camera, sparse_step=3, jointlimits=joint_limits)

    create_sparsespace(mesh)

    create_delaunaymesh_1DOF(mesh, 2)
    #create_delaunaymesh_2DOF(mesh,1)

if __name__ == '__main__':
    main()
