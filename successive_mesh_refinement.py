'''
May 16 2025 

This program uses successive mesh refinement and recursion, 
recursively refining local areas of the mesh until the 
error of the centroid of our mesh shape < RESTOL, the residual tolerance.

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
    points: tuple containing d np.arrays, each np.array is n-dim.
    for instance, 
    triangle in R2 : d=3, n=2, (x1+x2+x3 / 3, y2+y2+y3/3)
    triangle in Rn : d=3, n=n, ( xa1+xa2+xa3 / 3, xb1+xb2+xb3 /3, ... xn1+xn2+xn3/3)
    tetrahedral in R3 : d=4, n=3, (x1+x2+x3+x4 / 4, y2+y2+y3+y4/4,  z2+z2+z3+z4/4, w2+w2+w3+w4/4,)

    points.structure = x1,y1, x2,y2, x3,y3, etc. so for every dimension in Rn we need to extract the ith item, 
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

class Triangle:
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
        self.ddim = 4 #by definition of tetrahedral 
        self.ndim = point1.shape[0]
        points=(self.point1, self.point2, self.point3, self.point4)
        self.structure = points
        self.centroid = calculate_centroid(self)

class DelaunayMesh:
    def __init__(self, restol: float, robot: rtb.ERobot, camera: CentralCamera, sparse_step: int, mode: int): #using robots defined by ETS
        self.restol = restol
        self.robot = robot
        self.camera = camera #for now we wont use the camera. TODO: camera unused
        self.iterations=0 # incremented every time another point is plotted into our mesh
        self.nodes = [] #for one item in the list we have: xa1, xa2, ..., xan, position in real world (the last index may be modified)
        if robot.ets().n > 2: #number of joints/parameters the robot has
            self.shape = Tetrahedron #tuple of four np arrays. each np array is n-dimensional.
        else:
            self.shape = Triangle #tuple of three np arrays. each np array is 2 dimensional. pretty limited here. TODO: user choose triangle or tetrahedral?  
        if mode==1: #all axis are joint parameters, and number of meshing indicates how complex the change in parameters becomes at that region.
            pass #TODO: will this method even work? think on it
        if mode==2:
            #plot the joint parameters and the position we end up in. TODO: need to make an x,y,z subplot to show individual position 
            self.error

        self.sparse_step = sparse_step #initial grid size: how many datapoints for EACH q range? this is essentially the step size in the linear space 
    def plotmesh(self):
        #plot in real time or at the end? for now plot at end. TODO: implement plot
        pass
    def recursive_add_point(self, parent: Triangle):
        '''
        update iterations and nodes for each point that is added to the mesh '''
        #calculate the centroid. compare centroid to the real point at the specified angles.
        centroid = parent.centroid


    

ets = rtb.ET.Rz() * rtb.ET.tx(1) * rtb.ET.Rz() * rtb.ET.tx(1)
robot = rtb.Robot(ets)
print((robot.ets()).n)

mytet = Tetrahedron(np.array([1,2,3]),np.array([4,5,6]),np.array([7,8,9]),np.array([10,11,12]))
print(mytet.centroid)


