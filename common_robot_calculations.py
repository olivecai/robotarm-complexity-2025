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

def fkin(q, ets):
    T = ets.eval(q)
    pos = T[:3, 3]  # safer and clearer
    return pos

def centraldiff_jacobian(currQ, e: rtb.Robot.ets):
    '''
    Initialize and return a Jacobian through central differences, without use of a camera.'''
    epsilon=1e-5
     ###sanity check for our dimensions: 
    # let p=robot params, k=datapts for overdet system. 
    # each dP=1xd, each dQ=1xp. after generating overdet sys, 
    # DP=kxd, DQ=kxp, currJ.T=pxd... so currJ=dxp. 
    # Also, I is pxp
    # in this case, p=3, d=2
    p=currQ.shape[0]
    d=fkin(currQ, e).shape[0]
 #d is world dimensions
    k=1 #for now its not overdetermined. NOTE: find out if Dylan's paper used overdet.
    currJT = np.zeros((p,d)) 
    I=np.identity(p)
    for i in range(p):
        forward = fkin(currQ + epsilon * I[i], e)
        backward = fkin(currQ - epsilon * I[i], e)
        currJT[i] = (forward - backward) / (2 * epsilon)

    return currJT.T

def analytic_jacobian(q: np.ndarray, ets: rtb.ETS) -> np.ndarray:
    '''
    Computes the analytic Jacobian of a robot defined by an ETS at specific configuration q.

    Returns:
        np.ndarray: The Jacobian matrix (6 x N), where N is the number of joints.
    '''
    robot = rtb.ERobot(ets)
    return robot.jacob0(q)[:3, :]

