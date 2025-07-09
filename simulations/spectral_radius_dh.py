'''
July 9 

Generate the spectral radius for any Denavit Hartenberg robot...
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
import successive_mesh_refinement as smr
import common_robot_calculations as crc

####### GLOBAL ##########
t = sp.symbols('t(0:8)', real=True)
t0,t1,t2,t3,t4,t5,t6,t7 = t


#get the fkinematics equations...

class SpectralRadius:
    def __init__(self, fkin_equations, dof):
        self.fkin_eq = fkin_equations #fkin_equations == ee_translation, it is simply the real world POSITION/translation vector, NO ROTATION
        self.dof = dof
        self.spectral_radius= self.calc_sr
    
    def calc_sr(self, fkin_eq):
        f1,f2,f3 = self.fkin_eq
        u = sp.symbols('u(0:8)', real=True)   #u0,u1,u2,u3,u4,u5,u6,u7 
        I = sp.eye(3)
        f = sp.Matrix([f1,f2,f3])
        J = f.jacobian([])
        
    

class DenavitHartenbergSystem:
    def __init__(self, dh_params: list):
        #dh_params is a double nested list, where each row is one joint.
        transforms=[]
        for param in dh_params:
            transforms.append(self.transformation_matrix_DH(*param))
        ee_matrix = sp.eye(4)
        for i in range(len(transforms)):
            ee_matrix *= transforms[i]

        ee_translation = ee_matrix[:,3]
        self.ee_matrix = ee_matrix
        self.ee_translation = ee_translation
        self.dof = len(dh_params)

        self.spectral_radius = self.calc_spectral_radius()

    def calc_spectral_radius(self):
        
        f1,f2,f3,_ = self.ee_translation
        if self.dof == 2:
            I=sp.eye(2)
            f=f1,f2
        else:
            I=sp.eye(3)
            f=f1,f2,f3
            
        u = sp.symbols(f'u(0:{self.dof})', real=True)   #u0,u1,u2,u3,u4,u5,u6,u7, ... u(self.dof-1)
        
   
        f = sp.Matrix([f])
        J = f.jacobian([t[:self.dof]])

        reps_des=[]
        for i in range(self.dof):
            reps_des.append((t[i], u[i]))

        print("fkin:\n",f)
        print("J:\n",J)

        try:
            B = J.inv()
            print("Successfully computed inverse.")
            print("B:\n", B)
        except:
            print("Error calcualting inverse!")
            B = None
       
        evals=None
        sr=None

        if B is not None:
            dF = J.subs(reps_des)

            A = I - B*dF

            print("( del gn / del qn ) (aka A):\n", A)


            # Compute matrix norms symbolically
            norm_1 = A.norm(1)      # max column sum
            norm_inf = A.norm(sp.oo)  # max row sum
            norm_frob = A.norm('fro')

            print("1-norm upper bound on spectral radius:\n", norm_1)
            print("∞-norm upper bound on spectral radius:\n", norm_inf)
            print("Frobenius norm upper bound on spectral radius:\n", norm_frob)

            norm_2 = A.norm(2) #spectral norm
            print("spectral norm:\n", norm_2)

            #evals, sr=self.evals_and_sr(A)


        #print("SPECTRAL RADIUS:\n", sr)
        #print("E-VALS:\n", evals)
    
    def evals_and_sr(self, A):
    
        # Get the eigenvalues
        try:
            eigenvals = A.eigenvals()  # returns dict: {eigenvalue: multiplicity}
            # Compute the spectral radius (max absolute eigenvalue)
            return list(eigenvals.keys()), sp.Max(*[sp.Abs(lam) for lam in eigenvals.keys()])
        except:
            return None, None


    def transformation_matrix_DH(self, theta_i, alpha_i, r_i, d_i):
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
        DH = general_DH.subs([(alpha, alpha_i), (r, r_i), (theta, theta_i), (d, d_i)])
        #print(DH)
        return DH
    
    def subs_params(self, reps:list):
        return(self.ee_translation.subs(reps))


dof2_params = [
                [t0, 0, 1, 0], 
                [t1, 0, 1, 0]
                ]

dylan_dof3_params=[
                [ t0, sp.pi/2, 0 , 0 ],
                [ t1,  0  ,  0.55, 0 ],
                [ t2,  0  ,  0.30, 0 ]
                ]

kinova_dof6_params = [[0, 0.,0.,sp.pi],
                       [t0, 0., -(156.43 + 128.38), sp.pi/2],
                       [t1-sp.pi/2, 410., -5.38, sp.pi],
                       [t2-sp.pi/2, 0., -6.38, sp.pi/2],
                       [t3+sp.pi, 0., -(208.43+105.93), sp.pi/2],
                       [t4+sp.pi, 0., 0., sp.pi/2],
                       [t5+sp.pi, 0., -(105.93+61.53), sp.pi]
                       ] #as written in the documentation but im pretty sure this is WRONG. alpha and d are certainly the wrong column.


kinova_6dof = DenavitHartenbergSystem(kinova_dof6_params)
#dof2 = DenavitHartenbergSystem(dof2_params)
#dylan_dof3 = DenavitHartenbergSystem(dylan_dof3_params)

def analyze(sp_eq):
    '''
    Given a sympy analytic equation, find out...
    when less than 1
    when inf
    '''
    pass

