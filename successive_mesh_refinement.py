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

X = 0 #first coord, TODO: we should ideally be able to consider all dimensions concurrently and use that error.

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

def newpoint(Shape, ith, adjustedcentroid):
    '''
    adjustedcentroid is the new point which we would like to make a new shape with. 
    ith is the point of the parent shape which we exclude (and swap for adjustedcentroid). 
    creates and returns np.array, shape (ddim,).'''
    ret = np.zeros(Shape.ddim)
    j=0
    for i in range(Shape.ddim):
        if i!=ith:
            ret[j]= (Shape.structure[i])
            j+=1;
    ret[j] = adjustedcentroid
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
    def __init__(self, restol: float, robot: rtb.ERobot, camera: CentralCamera, sparse_step: int, mode: int , X: int, jointlimits=list ): #using robots defined by ETS
        self.restol = restol
        self.robot = robot
        self.q_count= robot.ets().n
        self.jointlimits = jointlimits #joint_limits = [(-np.pi, np.pi), (-np.pi/2, np.pi/2), (-1, 2)]  # example for 3 DOF
        self.camera = camera #for now we wont use the camera. TODO: camera unused
        self.iterations=0 # incremented every time another point is plotted into our mesh
        self.nodes = [] #ordered list of the nodes we currently have in the mesh. for one item in the list we have: xa1, xa2, ..., xan, position in real world (the last index may be modified, TBD)
        if self.q_count > 2: #number of joints/parameters the robot has
            self.shape = Tetrahedron #tuple of four np arrays. each np array is n-dimensional.
        else:
            self.shape = Triangle #the resulting plot would be q0,q1,posx OR posy 
        self.mode=mode
        self.X = X #X=0 is x, X=1 is y, X=2 is z. 
        self.sparse_step = sparse_step #initial grid size: how many datapoints for EACH q range? this is essentially the step size in the linear space 
    def plotmesh(self):
        #plot in real time/steps or at the end? for now plot at end of iterations. TODO: implement plot
        pass

def recursive_add_triangles(mesh: DelaunayMesh, parent: Triangle):
    '''
    TRIANGLES
    update iterations and nodes for each point that is added to the mesh '''
    #calculate the centroid. compare centroid to the real point at the specified angles.
    centroid = parent.centroid 
    q = centroid[:mesh.q_count] #extract q, since the remaining elements in the array may be position coords
    posMesh = centroid[mesh.q_count] #only 1 dimension. always store the dimension as the last index of the position array of our nodes.
    posR = (forward_kin(q, mesh.robot.ets()))[X] #X=0 is x, X=1 is y, X=2 is z. 
    #TODO: FOR NOW, JUST LOOK AT the Xth dimension, which will be a global constant FOR NOW!!!!!!!!!!
    residual = np.linalg.norm(posR - posMesh) #using np linalg norm since we might want to expand later.
    if residual > mesh.restol: #then we should mesh again at the centroid and recurse on each child. for triangles, 3 children are created; tetrahedrons, 4.
        for i in parent.structure: #ie for i in range(parent.ddim)
            centroid[mesh.q_count] = posR #if the residual is larger than restol, we should refine there.
            newpnt= newpoint(parent, i, centroid)
            mesh.iterations+=1
            mesh.nodes.append(newpnt) #just keeping track of things :-) 
            child=Triangle(newpnt[0], newpnt[1], newpnt[2])  
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
    posMesh = centroid[mesh.q_count] #only 1 dimension. always store the dimension as the last index of the position array of our nodes.
    posR = (forward_kin(q, mesh.robot.ets()))[X] #X=0 is x, X=1 is y, X=2 is z. 
    #TODO: FOR NOW, JUST LOOK AT the Xth dimension, which will be a global constant FOR NOW!!!!!!!!!!
    residual = np.linalg.norm(posR - posMesh) #using np linalg norm since we might want to expand later.

    if residual > mesh.restol: #then we should mesh again with the centroid q and pos as the real pos and recurse on each child. for triangles, 3 children are created; tetrahedrons, 4.
        for i in parent.structure: #ie for i in range(parent.ddim)
            centroid[mesh.q_count] = posR
            newpnt= newpoint(parent, i, centroid)
            mesh.iterations+=1
            mesh.nodes.append(newpnt) #just keeping track of things :-) 
            child=Tetrahedron(newpnt[0], newpnt[1], newpnt[2], newpnt[3])  
            return recursive_add_tetrahedrons(mesh, child)    
    else:
        return 
    
def create_sparsespace(Mesh: DelaunayMesh):
    '''
    1. generate linspace for each dof
    2. create a meshgrid and plot our points.
    3. if tetrahedrons: for every four points make shape & call recursive_add_tetrahedrons
        OR triangles: for every three points, call recursive_add_triangles
    4. our recursive mesh helper fns add points into our list of points as needed. 
    5. by the end of the recursive steps, we should have a list with many different points in it.
    6. pass this list into scipy delaunay mesh.
    7. NOTE: as we recurse, we create the notion of triangles/tetrahedrals in order to find the centroid. 
        However, it is not for CERTAIN (as far as i currently know) that scipy will create mesh with the same triangles; 
        ie a better mesh may be available after obtaining all points. This is fine. 
        We simply keep the notion of triangles in order to find the centroid.
    '''
    n = Mesh.sparse_step #resolution. NOTE: should be very low, maybe n=2
    # create linspaces and meshgrid
    joint_ranges = [np.linspace(low, high, n) for (low, high) in Mesh.jointlimits]
    grid = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(grid, axis=-1)  # shape: (n, n, ..., n, dof)

    # preallocate error arrays (same shape as grid, but without dof)
    error_shape = Q_grid.shape[:-1]
    TotalError = np.zeros(error_shape)

    for idx in np.ndindex(error_shape):
                Q = Q_grid[idx] 
                meshpoint = np.zeros(Q.shape[0] + 1)
                posR = forward_kin(q=Q, ets= Mesh.robot.ets())
                meshpoint[: Q.shape[0]] = Q
                meshpoint[Q.shape[0]] = posR[mesh.X]
                print(meshpoint)

                mesh.nodes.append(meshpoint)
    
    TODO: for every n nodes, create a shape and perform recursion 
    ie if tetrahedron: create tetrahedron for every four successive nodes and then exchange first node for the next to repeat
    all we really need to do is pass it to recursive function and keep updating the mesh.nodes list and then at the end of it all, run delaunay meshing and view
    how can we get a good idea of whats going on in higher dof?

    

ets = rtb.ET.Rz() * rtb.ET.tx(1) * rtb.ET.Rz() * rtb.ET.tx(1)
camera = CentralCamera()
robot = rtb.Robot(ets)
print((robot.ets()).n)

mytet = Tetrahedron(np.array([1,2,3]),np.array([4,5,6]),np.array([7,8,9]),np.array([10,11,12]))
print(mytet.centroid[:2])


joint_limits = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2)]  # example for 3 DOF
mesh = DelaunayMesh(1e-4, robot, camera, sparse_step=3, mode=2,  X=0, jointlimits=joint_limits)

create_sparsespace(mesh)


