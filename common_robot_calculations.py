'''
May 30 2025

Some common functions like forward kinematics and jacobian calculations are very common, 
so it would be nice to store them here.

'''

import numpy as np
from spatialmath import SE3
from spatialmath.base import e2h, h2e
from machinevisiontoolbox import CentralCamera
import roboticstoolbox as rtb

PI = np.pi

def fkin(q, ets: rtb.ETS):
    '''
    return the real world/actual position in R3 of the end effector based on the specified joint parameters, q (np array).
    '''
    T = ets.eval(q)       
    x=T[0][3];
    y=T[1][3];
    z=T[2][3];
    return np.array([x,y,z]);

def centraldiff_jacobian(currQ, e: rtb.Robot.ets):
    '''
    Initialize and return a Jacobian through central differences, without use of a camera.'''
    epsilon=PI/16
     ###sanity check for our dimensions: 
    # let p=robot params, k=datapts for overdet system. 
    # each dP=1xd, each dQ=1xp. after generating overdet sys, 
    # DP=kxd, DQ=kxp, currJ.T=pxd... so currJ=dxp. 
    # Also, I is pxp
    # in this case, p=3, d=2
    p=currQ.shape[0]
    d=3 #d is world dimensions
    k=1 #for now its not overdetermined. NOTE: find out if Dylan's paper used overdet.
    currJT = np.zeros((p,d)) 
    I=np.identity(p)
    for i in range(p): #i is a specific robot param
        #the jacobian is first initialized with central differences. 
        '''print("INIT JAC: currQ: ")
        print(currQ)
        print(I[i])
        print("positive perturb", vs_fkin(e, currQ+epsilon*I[i], camera))
        print("negative perturb", vs_fkin(e, currQ-epsilon*I[i], camera))
        print("perturb diff",  (vs_fkin(e, currQ+epsilon*I[i], camera)) - (vs_fkin(e, currQ-epsilon*I[i], camera)))'''
        currJT[i] = (
                    (fkin(currQ+epsilon*I[i], e)) -
                    (fkin(currQ-epsilon*I[i], e))
                ) / (2*epsilon)
        #print("currJT i column", currJT[i])

    currJ=currJT.T
    return currJ

def analytic_jacobian(q: np.ndarray, ets: rtb.ETS) -> np.ndarray:
    '''
    Computes the analytic Jacobian of a robot defined by an ETS at specific configuration q.

    Returns:
        np.ndarray: The Jacobian matrix (6 x N), where N is the number of joints.
    '''
    robot = rtb.Robot(ets)
    return robot.jacobe(q)[:3, :]

