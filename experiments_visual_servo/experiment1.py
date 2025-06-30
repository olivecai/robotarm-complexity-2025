

'''
June 30 2025

Use CAMERA to project robot onto image
Use OPENCV to draw radius of movement around second last joint
Use 
'''
import numpy as np
import math
from spatialmath import SE3
from spatialmath.base import e2h, h2e
import machinevisiontoolbox as mvtb
from machinevisiontoolbox import CentralCamera
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go

import sympy as sp
#import simulations.successive_mesh_refinement as smr
#import simulations.common_robot_calculations as crc

class Robot:
    def __init__(self, DH_parameter_matrix):
        
        self.DH_parameter_matrix = DH_parameter_matrix
        self.DH_transformation_matrices = []
        self.cartesian_translation_positions = []
        self.fkin_cartesian = None #return the position of every joint. idx 0 is base, idx len-1 is EE.
        self.DOF = None
        self.joint_limits = None

        for i in range(sp.shape(DH_parameter_matrix)[0]):
            print(DH_parameter_matrix.row(i))
            self.DH_transformation_matrices.append(self.calc_trans_mat_DH(DH_parameter_matrix.row(i)))

        print(self.DH_transformation_matrices)
        
        T_current = sp.eye(4)

        for T in self.DH_transformation_matrices:
            T_current = T_current * T
            pos=(T_current)
            pos = np.transpose(T_current[:,3])[0][:3]
            self.cartesian_translation_positions.append(pos)
            
        self.cartesian_translation_positions = sp.Matrix(self.cartesian_translation_positions)

    def get_specific_pose(self, Q):
        reps = [(t0, Q[0]), (t1, Q[1]), (t2, Q[2])]
        print("poses at",Q,"is:",self.cartesian_translation_positions.subs(reps))
        return self.cartesian_translation_positions.subs(reps)
    
    def get_EE_world_pos(self,Q):
        reps = [(t0, Q[0]), (t1, Q[1]), (t2, Q[2])]
        world_EE= sp.Matrix(list(self.cartesian_translation_positions.row(-1).subs(reps))) #shape is [[x],[y],[z]], (3,1)
        print("EE at",Q,"is",world_EE)
        return np.array(world_EE)

    def calc_trans_mat_DH(self,DH_param_mat_row_i):
        '''
        Returns the general denavit hartenberg transformation matrix for one link, i.
        theta_i is the angle about the z(i-1) axis between x(i-1) and x(i).
        alpha_i is the angle about the x(i) axis between z(i-1) and z(i).
        r_i is the distance between the origin of frame i-1 and i along the x(i) direction.
        d_i is the distance from x(i-1) to x(i) along the z(i-1) direction.
        '''
        alpha, r, theta, d = sp.symbols('alpha, r, theta, d', real=True)
        general_DH = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), r*sp.cos(theta)],
                                [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), r*sp.sin(theta)],
                                [0, sp.sin(alpha), sp.cos(alpha), d],
                                [0,0,0,1]])
        #print(general_DH)
        theta_i, alpha_i, r_i, d_i = DH_param_mat_row_i
        DH = general_DH.subs([(alpha, alpha_i), (r, r_i), (theta, theta_i), (d, d_i)])
        #print(DH)
        return DH
    
    def wireframe(self, joints):
        '''
        This function serves to plot the wireframe of our robot points.
        Input is a series of points, which will be connected in the plot sequentially.
        Fixed axis limits will prevent automatic scaling perspective.
        '''
        print(joints)
        x = joints[:,0]
        y = joints[:,1]
        z = joints[:,2]

        fig=plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.plot(x,y,z, marker='o', linestyle='-')

        xlim=[-1,1]
        ylim=[-1,1]
        zlim=[-1,1]
        # Set fixed axis limits
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_zlim(zlim)

        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
        ax.set_title('3D Line Plot')
        plt.show()

class Camera:
    def __init__(self, ID, rot_xaxis, rot_yaxis, rot_zaxis, translation, fx, fy, cx, cy):
        '''
        rot axis is a radian rotation around a specified axis.
        '''
        self.ID = ID
        self.K=sp.Matrix([[fx, 0, cx],[0, fy, cy], [0,0,1]]) #intrinsic matrix

        rx = sp.Matrix([[1,0,0],[0,sp.cos(rot_xaxis), -sp.sin(rot_xaxis)],[0,sp.sin(rot_xaxis), sp.cos(rot_xaxis)]])
        ry= sp.Matrix([[sp.cos(rot_yaxis), 0, sp.sin(rot_yaxis)],[0,1,0],[-sp.sin(rot_yaxis), 0, sp.cos(rot_yaxis)]])
        rz = sp.Matrix([[sp.cos(rot_zaxis), -sp.sin(rot_zaxis), 0], [sp.sin(rot_zaxis), sp.cos(rot_zaxis),0],[0,0,1]])
        
        R = rx*ry*rz
        t=sp.Matrix([translation[0],translation[1],translation[2]])
        
        E = R.col_insert(3,t)

        self.E = E

        self.P = self.K*self.E

    def projectpoint(self, worldpoint):
        if sp.shape(worldpoint)[0] == 3:
            # add a row
            worldpoint = worldpoint.row_insert(worldpoint.rows, sp.Matrix([1]))
        print("P", self.P)
        x = self.P * worldpoint
        x[0]=x[0]/x[2]
        x[1]=x[1]/x[2]
        x[2]=1#'''
        return sp.Matrix([[x[0]],[x[1]]])
    
    def show_projection(self, joints):
        # project each joint into image and then plot the robot.
        joint_projections =[]
        for i in range(sp.shape(joints)[0]):
            print(joints.row(i))
            joint_projections.append(list(self.projectpoint(joints.row(i).T)))

        joint_projections=sp.Matrix(joint_projections)

        print("joint proj:", joint_projections)
        x = joint_projections[:,0]
        y = joint_projections[:,1]

        fig=plt.figure()
        ax = fig.add_subplot()
        ax.plot(x,y, marker='o', linestyle='-')

        xlim=[-1,1]
        ylim=[-1,1]
        zlim=[-1,1]
        # Set fixed axis limits
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)

        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_title(f"Camera {self.ID} Projection Line Plot")
        plt.show()

class VisualServo:
    def __init__(self, tolerance, maxiter, alpha, robot : Robot, cameras : list, desRP, currQ):
        self.tolerance = tolerance
        self.maxiter= maxiter
        self.alpha = alpha
        self.robot = robot
        self.cameras = cameras
        self.desRP = self.desRP
        self.DOF = sp.shape(currQ)[0]

        self.currQ = currQ
        self.currErr = self.get_curr_err()
        self.jacobian = np.zeros((2, self.DOF))

        self.visualservo() #begin visual servoing
    
    def joint_radius_visualservo(self):
        # generate equation for ellipse around last joint
        # calculate shortest distance from last elbow to goal
        pass
        
    def move_robot(self, dQ):
        self.currQ += dQ

    def get_currErr(self):
        '''
        get the error of xcam1, ycam1, xcam2, ycam2, etc...
        return a sp matrix of size (2*n, 1) where n is the number of cameras
        '''
        number_of_cameras = len(self.cameras)
        self.currErr = sp.zeros(2*number_of_cameras, 1)

    def central_differences_jacobian(self):
        #perturb each joint and record the corresponding central differences change
        delta = 0.025
        move = np.zeros(self.DOF)
        print("###### computing central differences #######")
        for i in range(self.DOF):
            initial_error=self.currErr
            self.robot.get_EE_world_pos(self.currQ)



def main():
    #get projected image of robot, planar for now

    #NOTE: we can perturb the robot to get not only jacobians but also the circles of movement... assume each dof lies on a 2D hyperplane.
    pass

cam1 = Camera(1, 0,0,0,[0,0,5], 5,5, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = Camera(2, -sp.pi/2, 0, 0, [0,0,5], 5,5,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 

t0,t1,t2 = sp.symbols('t(0:3)', real=True)
DH_matrix = sp.Matrix([[ t0, sp.pi/2,  0   , 0 ],
                       [ t1,  0  ,  0.55, 0 ],
                       [ t2,  0  ,  0.30, 0 ]])
robo = Robot(DH_matrix)

joints_to_wireframe =  robo.get_specific_pose([0,0,sp.pi/2])
robo.get_EE_world_pos([0,0,sp.pi/2])

robo.wireframe(joints_to_wireframe)

#cam1.show_projection(joints_to_wireframe)
#cam2.show_projection(joints_to_wireframe)

