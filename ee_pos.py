'''
May 23 2025
This is a very very simple program to illustrate end effector positions over DOF pairs calculated from forward kinematics.
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

ets = rtb.ET.Rz() * rtb.ET.tx(1) * rtb.ET.Rz() * rtb.ET.tx(1)
camera = CentralCamera()
robot = rtb.Robot(ets)

joint_limits = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2)]  # example for 2 DOF

def forward_kin(q, ets: rtb.ETS):
    '''
    return the real world/actual position in R3 of the end effector based on the specified joint parameters, q (np array).
    '''
    T = ets.eval(q)       
    x=T[0][3];
    y=T[1][3];
    z=T[2][3];
    return y;

 
def plotcoordpos(jointlimits: list, ets: rtb.ETS):
     # make a linspace and meshgrid for ranges of q0 and q1
    n = 100  #resolution
    q_range = np.linspace(-np.pi/2, np.pi/2, n)
    Q0, Q1 = np.meshgrid(q_range, q_range) 
    coordplot = np.zeros_like(Q0)

    # double loop thru q0 and q1, compute inverse kinematics at that starting point and compute the error
    for i in range(n):
        for j in range(n):
            q0 = Q0[i, j]
            q1 = Q1[i, j]
            coordplot[i,j] = forward_kin(np.array([q0,q1]), ets)

    fig = plt.figure(figsize=(6, 6))
    ax1 = fig.add_subplot(111, projection='3d')
    ax1.plot_surface(Q0, Q1, coordplot, cmap='plasma')
    ax1.set_xlabel("q0 (rad)")
    ax1.set_ylabel("q1 (rad)")
    ax1.set_zlabel("Y coord")
    ax1.set_title("Position")

    plt.show()

plotcoordpos(joint_limits, ets)